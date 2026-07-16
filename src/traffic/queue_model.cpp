#include <nomad/traffic/queue_model.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace nomad {

QueueTrafficModel::QueueTrafficModel(const Graph& graph) : graph_(graph) {
    const uint32_t E = graph.num_edges();
    queues_.resize(E);
    states_.resize(E);
    is_active_.assign(E, 0);
    active_list_.reserve(std::min(E, 100'000u));

    for (uint32_t e = 0; e < E; ++e) {
        const EdgeData& ed = graph.edges[e];
        float estimated_lanes = std::max(1.0f, ed.capacity / 1600.0f);
        // Minimum effective length of 50m: micro-segments (median ~16m) would otherwise
        // have storage_cap of 2-3 vehicles, triggering BPR congestion with just 1-2 agents.
        float eff_len = std::max(50.0f, ed.length_m);
        queues_[e].storage_cap    = eff_len * estimated_lanes * kJamDensity;
        // NOTE: headway-based discharge constraint is disabled pending calibration.
        // The BPR model already caps travel time at 3.4× free-flow (vc=2), which
        // keeps agents below the stuck_threshold_ratio=5 teleport threshold.
        // A proper kinematic-wave or CTM model is needed before re-enabling this.
        queues_[e].flow_cap_per_s = 0.0f;
        states_[e].travel_time_s.store(graph.free_flow_time(e), std::memory_order_relaxed);
    }
}

SimTime QueueTrafficModel::on_enter(EdgeId e, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return t + 1.0;
    LinkQueue& lq = queues_[e];
    ++lq.occupancy_count;
    float occ = static_cast<float>(lq.occupancy_count);
    states_[e].occupancy.store(occ, std::memory_order_relaxed);

    // Track newly-active edges for the dirty set.
    if (lq.occupancy_count == 1 && !is_active_[e]) {
        is_active_[e] = 1;
        active_list_.push_back(e);
    }

    float cap = lq.storage_cap;
    float ff  = graph_.free_flow_time(e);
    if (cap <= 0.0f || occ / cap < 0.8f) {
        states_[e].travel_time_s.store(ff, std::memory_order_relaxed);
        return t + ff;
    }
    // Density-based v/c ratio capped at 2× storage capacity.
    // This protects micro-links (ff < 5s): they need ≥8 physical vehicles before
    // BPR kicks in (80% of storage_cap based on jam density × effective length).
    // Cap at vc=2 → max tt = ff × (1 + 0.15 × 16) = ff × 3.4, preventing the
    // 20× trap that kept agents on congested links indefinitely.
    float vc = std::min(occ / cap, 2.0f);
    float tt = ff * (1.0f + 0.15f * std::pow(vc, 4.0f));
    states_[e].travel_time_s.store(tt, std::memory_order_relaxed);
    return t + tt;
}

bool QueueTrafficModel::on_exit(EdgeId e, EdgeId /*next*/, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return true;
    LinkQueue& lq = queues_[e];

    // Discharge-rate constraint: enforce minimum headway between consecutive exits.
    // Without this, the BPR delay model lets an entire queue clear in one timestep
    // (all agents scheduled at t+ff*BPR simultaneously), which is physically
    // impossible — a real link discharges at most flow_cap vehicles per second.
    // When blocked, caller reschedules at t+1 (see handle_exit_link in simulation.cpp).
    if (lq.flow_cap_per_s > 0.0f) {
        double headway = 1.0 / static_cast<double>(lq.flow_cap_per_s);
        if (t < lq.last_exit_time + headway)
            return false;
    }

    if (lq.occupancy_count > 0) --lq.occupancy_count;
    float occ = static_cast<float>(lq.occupancy_count);
    states_[e].occupancy.store(occ, std::memory_order_relaxed);
    lq.last_exit_time = t;
    states_[e].outflow_rate.store(lq.flow_cap_per_s, std::memory_order_relaxed);
    return true;
}

void QueueTrafficModel::force_remove(EdgeId e, AgentId /*a*/) {
    if (e >= graph_.num_edges()) return;
    LinkQueue& lq = queues_[e];
    if (lq.occupancy_count > 0) --lq.occupancy_count;
    states_[e].occupancy.store(
        static_cast<float>(lq.occupancy_count), std::memory_order_relaxed);
    // Recompute travel time immediately so the link no longer appears congested
    float occ = static_cast<float>(lq.occupancy_count);
    float cap = lq.storage_cap;
    float ff  = graph_.free_flow_time(e);
    if (cap <= 0.0f || occ / cap < 0.8f) {
        states_[e].travel_time_s.store(ff, std::memory_order_relaxed);
    } else {
        float vc = std::min(occ / cap, 2.0f);
        states_[e].travel_time_s.store(
            ff * (1.0f + 0.15f * std::pow(vc, 4.0f)), std::memory_order_relaxed);
    }
}

void QueueTrafficModel::update(SimTime /*t*/) {
    // Scan only active edges — O(active) instead of O(E).
    // Recompute travel_time_s from current occupancy so snapshots always reflect
    // the actual state (on_enter only updates TT when an agent arrives, leaving
    // a stale congested value if occupancy has since dropped).
    uint32_t out = 0;
    for (uint32_t i = 0; i < static_cast<uint32_t>(active_list_.size()); ++i) {
        EdgeId e   = active_list_[i];
        float  occ = static_cast<float>(queues_[e].occupancy_count);
        states_[e].occupancy.store(occ, std::memory_order_relaxed);
        float ff = graph_.free_flow_time(e);
        float tt;
        if (occ <= 0.0f) {
            tt = ff;
            is_active_[e] = 0;
        } else {
            // Recompute BPR travel time from current occupancy
            float cap = queues_[e].storage_cap;
            if (cap <= 0.0f || occ / cap < 0.8f) {
                tt = ff;
            } else {
                float vc = std::min(occ / cap, 2.0f);
                tt = ff * (1.0f + 0.15f * std::pow(vc, 4.0f));
            }
            active_list_[out++] = e;
        }
        states_[e].travel_time_s.store(tt, std::memory_order_relaxed);

        // Smooth tt/ff into the EMA so brief congestion spikes on
        // fast-clearing edges still register over several sync windows.
        if (ff > 0.0f) {
            float ratio = tt / ff;
            float ema   = states_[e].congestion_ema.load(std::memory_order_relaxed);
            states_[e].congestion_ema.store(
                ema + kCongestionEmaAlpha * (ratio - ema), std::memory_order_relaxed);
        }
    }
    active_list_.resize(out);
}

float QueueTrafficModel::current_travel_time(EdgeId e) const {
    if (e >= graph_.num_edges()) return 1.0f;
    return states_[e].travel_time_s.load(std::memory_order_relaxed);
}

std::span<const LinkState> QueueTrafficModel::link_states() const {
    return {states_.data(), states_.size()};
}

} // namespace nomad
