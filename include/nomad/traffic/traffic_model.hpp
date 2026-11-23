#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <atomic>
#include <span>
#include <string_view>
#include <vector>

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

// ── Counted per-edge throughput ───────────────────────────────────────────────
// Cumulative since simulation start, one entry per directed edge. Filled by
// the simulation engine (not by the traffic models) on every real car
// entry/exit, so it is identical for every ITrafficModel implementation.
// The two time sums give the exact vehicle-seconds spent on the edge over any
// window [t0, t1] without storing per-vehicle dwell intervals:
//   ∫ occ dt = Δsum_t_exit − Δsum_t_enter + occ(t1)·t1 − occ(t0)·t0
// with occ = n_enter − n_exit. t0/t1 must be ITrafficModel::counters_horizon()
// (the latest event time recorded so far), NOT the engine's window clock:
// events carry their own timestamps and a window's batch runs ahead of the
// clock value writers are handed, so only the horizon is guaranteed to be
// ≥ every recorded entry time (which keeps the integral non-negative).
struct EdgeCounters {
    uint32_t n_enter     = 0;    // vehicles that entered the edge
    uint32_t n_exit      = 0;    // vehicles that left it (incl. teleported)
    double   sum_t_enter = 0.0;  // Σ entry times [s]
    double   sum_t_exit  = 0.0;  // Σ exit times [s]
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

    // ── Movement-aware variants (default: fall back to the plain versions) ──
    // `next` is the edge the agent will take AFTER e (kInvalidEdge if e is
    // its last edge). Models with per-movement storage (LTM with
    // movement_storage) use it; the others ignore it.
    // `next_after` (optional) is the edge after `next`: models with movement
    // lookahead also limit the vehicles on e heading to (next, next_after).
    virtual bool has_capacity_for(EdgeId e, EdgeId next, EdgeId next_after = kInvalidEdge) const {
        (void)next; (void)next_after; return has_capacity(e);
    }
    // Admission of a departing vehicle onto its first edge e (next = second
    // edge, next_after = third). Models with node priority treat departures
    // as minor entries.
    virtual bool has_capacity_departure(EdgeId e, EdgeId next, EdgeId next_after = kInvalidEdge) const {
        return has_capacity_for(e, next, next_after);
    }
    // Exit from e into `next`, where `next_after` and `next_after2` are the
    // two edges the agent will take after `next` (used for the spillback
    // check on `next`).
    virtual bool on_exit_to(EdgeId e, EdgeId next, EdgeId next_after, AgentId a, SimTime t,
                            EdgeId next_after2 = kInvalidEdge) {
        (void)next_after; (void)next_after2; return on_exit(e, next, a, t);
    }
    // Record (or update, after a reroute) the movement e -> next (-> next_after)
    // of an agent currently on e. Called right after on_enter() and after a
    // mid-link reroute. Released by a successful exit or force_remove().
    virtual void assign_movement(EdgeId e, AgentId a, EdgeId next, EdgeId next_after = kInvalidEdge) {
        (void)e; (void)a; (void)next; (void)next_after;
    }
    // QNode node model: true if a link's exit buffer is FIFO per movement
    // (next link) rather than per link, i.e. a blocked movement does not
    // block vehicles heading elsewhere (multi-lane approach).
    virtual bool fifo_per_movement() const { return false; }
    // QNode node model: true if the per-movement FIFO is keyed by the lane
    // group (fifo_group), not by the next link.
    virtual bool fifo_lookahead() const { return false; }
    // QNode node model: serve major approaches before minor ones (give-way).
    virtual bool priority_service() const { return false; }
    virtual bool is_major_at_node(EdgeId e) const { (void)e; return true; }

    // ── Path-aware variants (used by the engine) ──
    // `after` = the edges the agent takes after e, in order (after[0] = next).
    // Defaults reduce to the movement-aware versions above.
    static EdgeId at(std::span<const EdgeId> after, std::size_t k) {
        return k < after.size() ? after[k] : kInvalidEdge;
    }
    virtual bool has_capacity_path(EdgeId e, std::span<const EdgeId> after) const {
        return has_capacity_for(e, at(after, 0), at(after, 1));
    }
    virtual bool has_capacity_departure_path(EdgeId e, std::span<const EdgeId> after) const {
        return has_capacity_departure(e, at(after, 0), at(after, 1));
    }
    virtual bool on_exit_path(EdgeId e, std::span<const EdgeId> after, AgentId a, SimTime t) {
        return on_exit_to(e, at(after, 0), at(after, 1), a, t, at(after, 2));
    }
    virtual void assign_movement_path(EdgeId e, AgentId a, std::span<const EdgeId> after) {
        assign_movement(e, a, at(after, 0), at(after, 1));
    }
    // QNode per-movement FIFO key of a vehicle on e (vehicles with the same
    // key queue behind each other). Default: the next link.
    virtual EdgeId fifo_group(EdgeId e, std::span<const EdgeId> after) const {
        (void)e; return at(after, 0);
    }

    virtual std::string_view model_name() const = 0;

    // ── Counted throughput (see EdgeCounters) ────────────────────────────────
    // Called by the simulation engine from its serial event loop — not
    // thread-safe, and not meant to be called by model implementations.
    void init_edge_counters(std::size_t num_edges) {
        if (edge_counters_.size() != num_edges) edge_counters_.assign(num_edges, EdgeCounters{});
    }
    void record_enter(EdgeId e, SimTime t) {
        if (e >= edge_counters_.size()) return;
        ++edge_counters_[e].n_enter; edge_counters_[e].sum_t_enter += t;
        if (t > counters_horizon_) counters_horizon_ = t;
    }
    void record_exit(EdgeId e, SimTime t) {
        if (e >= edge_counters_.size()) return;
        ++edge_counters_[e].n_exit; edge_counters_[e].sum_t_exit += t;
        if (t > counters_horizon_) counters_horizon_ = t;
    }
    // Empty until init_edge_counters() has been called.
    std::span<const EdgeCounters> edge_counters() const { return edge_counters_; }
    // Latest event time recorded so far (0 before the first event).
    SimTime counters_horizon() const { return counters_horizon_; }

private:
    std::vector<EdgeCounters> edge_counters_;
    SimTime counters_horizon_ = 0.0;
};

} // namespace nomad
