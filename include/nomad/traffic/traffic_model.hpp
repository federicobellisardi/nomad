#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <atomic>
#include <span>
#include <string_view>

namespace nomad {

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

    LinkState() = default;
    LinkState(const LinkState& o) noexcept
        : occupancy    (o.occupancy    .load(std::memory_order_relaxed))
        , inflow_rate  (o.inflow_rate  .load(std::memory_order_relaxed))
        , outflow_rate (o.outflow_rate .load(std::memory_order_relaxed))
        , travel_time_s(o.travel_time_s.load(std::memory_order_relaxed)) {}
    LinkState(LinkState&& o) noexcept
        : occupancy    (o.occupancy    .load(std::memory_order_relaxed))
        , inflow_rate  (o.inflow_rate  .load(std::memory_order_relaxed))
        , outflow_rate (o.outflow_rate .load(std::memory_order_relaxed))
        , travel_time_s(o.travel_time_s.load(std::memory_order_relaxed)) {}
    LinkState& operator=(const LinkState& o) noexcept {
        occupancy    .store(o.occupancy    .load(std::memory_order_relaxed));
        inflow_rate  .store(o.inflow_rate  .load(std::memory_order_relaxed));
        outflow_rate .store(o.outflow_rate .load(std::memory_order_relaxed));
        travel_time_s.store(o.travel_time_s.load(std::memory_order_relaxed));
        return *this;
    }
};

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

    // Called when an agent's scheduled exit event fires.
    // Returns true if the agent can leave (downstream not full).
    // Returns false if the agent is blocked by spillback — the simulation
    // engine will reschedule the exit event 1s later.
    virtual bool on_exit(EdgeId e, AgentId a, SimTime t) = 0;

    // Called once per sync window after all events have been processed.
    // Use to update rolling averages, recompute densities, etc.
    virtual void update(SimTime t) = 0;

    // Current travel time used by the routing layer [s]. This is the value
    // the CH router uses for dynamic re-routing cost updates.
    virtual float current_travel_time(EdgeId e) const = 0;

    // Read-only view of link states for output writers and Python bindings
    virtual std::span<const LinkState> link_states() const = 0;

    virtual std::string_view model_name() const = 0;
};

} // namespace nomad
