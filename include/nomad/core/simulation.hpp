#pragma once

#include <nomad/core/agent.hpp>
#include <nomad/core/event.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
namespace nomad {
class ITrafficModel;
class IRouter;
class IDemandModel;
class IOutputWriter;
class AStarRouter;
}

namespace nomad {

// ── Simulation configuration ──────────────────────────────────────────────────
struct SimulationConfig {
    SimTime  start_time          = 0.0;    // seconds since midnight
    SimTime  end_time            = 86400.0;
    float    sync_window_s       = 30.0f;  // traffic state sync / writer poll period
    float    reroute_thresh        = 0.25f;  // re-route if tt > (1+thresh)*freeflow
    float    reroute_interval_s    = 60.0f;  // minimum simulated seconds between reroute scans
    float    stuck_threshold_ratio      = 20.0f;  // teleport if elapsed > ratio × freeflow_route (0 = disabled)
    float    teleport_interval_s        = 600.0f; // minimum simulated seconds between teleport scans
    uint32_t max_reroutes               = 10;     // stop rerouting an agent after this many reroutes (0 = unlimited)
    float    stuck_max_hours            = 4.0f;   // hard cap: teleport after this many hours regardless of route length (0 = disabled)
    float    route_randomization_sigma  = 0.0f;   // logNormal sigma for stochastic pre-routing (0 = CH only)
    float    walk_speed_ms       = 1.39f;  // propagated from ScenarioConfig::mode_choice
    float    bike_speed_ms       = 4.17f;  // propagated from ScenarioConfig::mode_choice
    uint32_t num_threads         = 0;      // 0 = hardware_concurrency
    bool     store_traces        = false;
    std::string traffic_model    = "queue";
    std::string router           = "CH";
    uint32_t snapshot_interval_s = 300;
};

// ── Simulation engine ─────────────────────────────────────────────────────────
// Hybrid DES with synchronisation windows.
//
// Algorithm per window [t, t + sync_window_s):
//   1. drain_until(t + sync_window_s) → batch of events
//   2. tbb::parallel_for(event in batch): handle_event(event)
//      — per-link mutex serialises same-link events
//   3. flush_traffic_state() — atomic snapshot
//   4. fire SnapshotDump hook
//   5. schedule re-routes for links exceeding reroute_thresh
//   6. advance current_time_ by sync_window_s
class Simulation {
public:
    explicit Simulation(SimulationConfig cfg = {});
    ~Simulation();

    // ── Setup (call before run()) ─────────────────────────────────────────────
    void set_graph(std::shared_ptr<Graph> g);
    void set_demand(std::shared_ptr<IDemandModel> d);
    void set_traffic_model(std::unique_ptr<ITrafficModel> m);
    void set_router(std::unique_ptr<IRouter> r);
    void add_output_writer(std::unique_ptr<IOutputWriter> w);

    // Register a callback invoked whenever an event of the given type fires.
    // Callbacks run on the thread pool — must be thread-safe.
    // Use a mutex or atomic if state is shared with Python.
    void register_hook(EventType type, std::function<void(const Event&)> cb);

    // ── Execution ─────────────────────────────────────────────────────────────
    void run();                      // run to cfg_.end_time
    void run_until(SimTime t);       // run until t (useful for Python step-by-step)
    void step();                     // advance one sync_window_s

    // ── State queries (thread-safe) ───────────────────────────────────────────
    SimTime     current_time()      const noexcept { return current_time_.load(); }
    std::size_t active_agents()     const noexcept;
    std::size_t events_processed()  const noexcept { return events_processed_.load(); }

    // Expose internal state (read-only, for Python zero-copy views)
    const AgentHotStore& agent_hot()    const noexcept { return hot_; }
    const AgentColdStore& agent_cold()  const noexcept { return cold_; }
    const RouteStore&    route_store()  const noexcept { return routes_; }
    const Graph&         graph()        const;
    const ITrafficModel& traffic()      const;

    // Read-only capability checks. compute_route() silently returns an empty
    // route whenever router_ is null (see simulation.cpp), so a Simulation
    // driven without ever calling set_router()/set_traffic_model() "runs"
    // to completion without routing or moving a single agent instead of
    // erroring. Added so callers (in particular the Python bindings) can
    // fail loudly before run()/run_until()/step() instead of silently
    // producing an empty result.
    bool has_router()        const noexcept { return router_  != nullptr; }
    bool has_traffic_model() const noexcept { return traffic_ != nullptr; }

private:
    // ── Event handlers ────────────────────────────────────────────────────────
    void handle_event            (const Event& e);
    void handle_depart           (const Event& e);
    void handle_enter_link       (const Event& e);
    void handle_exit_link        (const Event& e);
    void handle_arrive_activity  (const Event& e);
    void handle_reroute          (const Event& e);
    void handle_signal_change    (const Event& e);

    void inject_demand              ();
    void schedule_signal_changes    ();
    void flush_traffic_state        ();
    void schedule_reroutes          ();
    void teleport_stuck_agents      (SimTime now);
    void fire_hooks                 (EventType type, const Event& e);

    // Compute next route for an agent (CH or A* depending on router_)
    std::vector<EdgeId> compute_route(AgentId a);

    SimulationConfig   cfg_;

    std::shared_ptr<Graph>              graph_;
    std::shared_ptr<IDemandModel>       demand_;
    std::unique_ptr<ITrafficModel>      traffic_;
    std::unique_ptr<IRouter>            router_;
    // Traffic-aware A* used exclusively for en-route rerouting (CH keeps free-flow weights).
    std::unique_ptr<AStarRouter>        reroute_router_;
    std::vector<std::unique_ptr<IOutputWriter>> writers_;

    EventQueue     eq_;
    AgentHotStore  hot_;
    AgentColdStore cold_;
    RouteStore     routes_;

    std::atomic<SimTime>     current_time_{0.0};
    std::atomic<std::size_t> events_processed_{0};
    std::atomic<uint64_t>    n_depart_{0}, n_enter_{0}, n_exit_{0}, n_arrive_{0};
    std::atomic<uint64_t>    n_teleported_{0};

    // Diagnostics: reroute_count / road_class of every agent at the moment it
    // is teleported (captured in teleport_stuck_agents before state is
    // overwritten) — used to check whether teleported agents are routing-loop
    // victims (reroute_count saturated at cfg_.max_reroutes) or something else.
    std::array<uint64_t, 32>  teleported_by_reroute_count_{};
    std::array<uint64_t, 256> teleported_by_class_{};

    SimTime last_reroute_t_{-1e9f};
    SimTime last_teleport_t_{-1e9f};

    std::unordered_map<uint8_t, std::vector<std::function<void(const Event&)>>> hooks_;

    // TBB task arena (initialised in constructor)
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace nomad
