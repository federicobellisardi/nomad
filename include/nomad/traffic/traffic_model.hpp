#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <atomic>
#include <span>
#include <string_view>

namespace nomad {

// Jam density: 1 vehicle per 7.5m of queued spacing. Shared by every traffic
// model so storage-capacity accounting (and hence spillback behaviour) is
// consistent regardless of which model is selected.
inline constexpr float kJamDensity = 1.0f / 7.5f;

// ── Link state (per directed edge, updated each sync window) ──────────────────
// Stored as atomic floats so the routing layer can sample current travel times
// without acquiring per-link locks.
// Explicit move/copy constructors are required because std::atomic is neither
// copy- nor move-constructible, yet std::vector needs them for reallocation.
struct alignas(16) LinkState {
    std::atomic<float> occupancy{0.0f};
    std::atomic<float> inflow_rate{0.0f};
    std::atomic<float> outflow_rate{0.0f};
    std::atomic<float> travel_time_s{0.0f};
    // Exponential moving average of tt/ff, updated once per sync window
    // (~10 samples per reroute_interval_s). Smooths out short, bursty
    // congestion spikes on tiny/fast-clearing edges (e.g. motorway junction
    // connectors) that an instantaneous tt/ff read at the reroute-scan
    // instant would very likely miss. Used by Simulation::schedule_reroutes
    // instead of the instantaneous travel_time_s so agents whose bottleneck
    // is a frequently-but-not-always congested micro-edge actually get
    // flagged for rerouting. 1.0 = free-flow.
    std::atomic<float> congestion_ema{1.0f};

    LinkState() = default;
    LinkState(const LinkState& o) noexcept
        : occupancy      (o.occupancy      .load(std::memory_order_relaxed))
        , inflow_rate    (o.inflow_rate    .load(std::memory_order_relaxed))
        , outflow_rate   (o.outflow_rate   .load(std::memory_order_relaxed))
        , travel_time_s  (o.travel_time_s  .load(std::memory_order_relaxed))
        , congestion_ema (o.congestion_ema .load(std::memory_order_relaxed)) {}
    LinkState(LinkState&& o) noexcept
        : occupancy      (o.occupancy      .load(std::memory_order_relaxed))
        , inflow_rate    (o.inflow_rate    .load(std::memory_order_relaxed))
        , outflow_rate   (o.outflow_rate   .load(std::memory_order_relaxed))
        , travel_time_s  (o.travel_time_s  .load(std::memory_order_relaxed))
        , congestion_ema (o.congestion_ema .load(std::memory_order_relaxed)) {}
    LinkState& operator=(const LinkState& o) noexcept {
        occupancy     .store(o.occupancy     .load(std::memory_order_relaxed));
        inflow_rate   .store(o.inflow_rate   .load(std::memory_order_relaxed));
        outflow_rate  .store(o.outflow_rate  .load(std::memory_order_relaxed));
        travel_time_s .store(o.travel_time_s .load(std::memory_order_relaxed));
        congestion_ema.store(o.congestion_ema.load(std::memory_order_relaxed));
        return *this;
    }
};

// Smoothing factor for congestion_ema: ema += kCongestionEmaAlpha * (ratio - ema).
// Shared so both traffic models produce a comparable signal.
inline constexpr float kCongestionEmaAlpha = 0.2f;

// ── ITrafficModel ─────────────────────────────────────────────────────────────
// Primary extensibility seam. Implementations:
//   QueueTrafficModel — MATSim-style queue (default, efficient)
//   LtmTrafficModel   — Yperman 2007 LTM (accurate, corridor studies)
//
// Thread-safety contract:
//   on_enter / on_exit are called concurrently from the TBB thread pool.
//   Each implementation must serialise access to shared link state.
//   The LinkState atomics provide lock-free reads for routing.
class ITrafficModel {
public:
    virtual ~ITrafficModel() = default;

    // Called when an agent enters an edge. Returns the predicted travel time [s]
    // under current conditions (≥ free-flow time). The agent's exit event will
    // be scheduled at t + return_value.
    virtual SimTime on_enter(EdgeId e, AgentId a, SimTime t) = 0;

    // Called when an agent's scheduled exit event fires. `next` is the edge
    // the agent will enter immediately afterwards, or kInvalidEdge if this is
    // the agent's last edge (trip ends here — always allowed to exit).
    // Returns true if the agent can leave. Returns false if blocked by
    // spillback on `next` (downstream link has no storage capacity left) —
    // the simulation engine will reschedule the exit event 1s later.
    virtual bool on_exit(EdgeId e, EdgeId next, AgentId a, SimTime t) = 0;

    // Called once per sync window after all events have been processed.
    // Use to update rolling averages, recompute densities, etc.
    virtual void update(SimTime t) = 0;

    // Current travel time used by the routing layer [s]. This is the value
    // the CH router uses for dynamic re-routing cost updates.
    virtual float current_travel_time(EdgeId e) const = 0;

    // Read-only view of link states for output writers and Python bindings
    virtual std::span<const LinkState> link_states() const = 0;

    // Forcibly remove an agent from a link (teleport). Unlike on_exit(), this
    // always succeeds and does not enforce flow capacity. Used by the stuck-agent
    // teleport mechanism to clear agents that have exceeded their time budget.
    virtual void force_remove(EdgeId e, AgentId a) { (void)e; (void)a; }

    // Returns true if edge `e` currently has spare storage for a new entrant.
    // Used only by Simulation::handle_depart() to gate a trip's *first* edge:
    // there is no preceding on_exit() call for a trip origin (no "previous"
    // edge), so on_enter() would otherwise be reachable with unbounded
    // occupancy. Every subsequent edge transition is already gated by
    // on_exit()'s spillback check on `next` before handle_exit_link() pushes
    // the corresponding AgentEnterLink event. Default true: models without a
    // storage/spillback concept (QueueTrafficModel) accept unconditionally.
    virtual bool has_capacity(EdgeId e) const { (void)e; return true; }

    virtual std::string_view model_name() const = 0;
};

} // namespace nomad
