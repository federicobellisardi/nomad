#include <nomad/traffic/ltm_model.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace nomad {

LtmTrafficModel::LtmTrafficModel(const Graph& graph)
    : LtmTrafficModel(graph, Config{}) {}

LtmTrafficModel::LtmTrafficModel(const Graph& graph, Config cfg)
    : graph_(graph), cfg_(cfg)
{
    const uint32_t E = graph.num_edges();
    cells_.resize(E);
    states_.resize(E);
    is_active_.assign(E, 0);
    active_list_.reserve(std::min(E, 100'000u));

    for (uint32_t e = 0; e < E; ++e) {
        const EdgeData& ed = graph.edges[e];
        float estimated_lanes = std::max(1.0f, ed.capacity / 1600.0f);
        // Same 50m effective-length floor as QueueTrafficModel: micro-segments
        // (median ~16m) would otherwise have a storage capacity of 2-3
        // vehicles, triggering spillback from a single agent.
        float eff_len = std::max(50.0f, ed.length_m);
        cells_[e].storage_veh    = eff_len * estimated_lanes * cfg_.jam_density_vpm;
        cells_[e].capacity_veh_s = ed.capacity / 3600.0f;
        states_[e].travel_time_s.store(graph.free_flow_time(e), std::memory_order_relaxed);
    }
}

float LtmTrafficModel::travel_time_for(EdgeId e) const {
    const Cell& c = cells_[e];
    float ff  = graph_.free_flow_time(e);
    float cap = c.storage_veh;
    float occ = static_cast<float>(c.occupancy);
    if (cap <= 0.0f || occ / cap < 0.8f) return ff;
    // Same BPR volume-delay shape as QueueTrafficModel, capped at vc=2 → 3.4× ff.
    // Reusing it here isolates the effect under test to the new exit-side
    // spillback gate rather than also changing the delay formula.
    float vc = std::min(occ / cap, 2.0f);
    return ff * (1.0f + 0.15f * std::pow(vc, 4.0f));
}

SimTime LtmTrafficModel::on_enter(EdgeId e, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return t + 1.0;
    Cell& c = cells_[e];
    ++c.occupancy;
    states_[e].occupancy.store(static_cast<float>(c.occupancy), std::memory_order_relaxed);

    if (c.occupancy == 1 && !is_active_[e]) {
        is_active_[e] = 1;
        active_list_.push_back(e);
    }

    float tt = travel_time_for(e);
    states_[e].travel_time_s.store(tt, std::memory_order_relaxed);
    return t + tt;
}

bool LtmTrafficModel::on_exit(EdgeId e, EdgeId next, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return true;
    Cell& c = cells_[e];

    // Receiving-side spillback only: block if the downstream link has no
    // spare storage. This is the behaviour QueueTrafficModel does not have —
    // it lets congestion genuinely propagate backward instead of only
    // raising the current link's own travel time.
    //
    // Deliberately NOT also enforcing a per-link discharge headway here:
    // combined with per-link storage caps (which are already small — a
    // 50m/~7-vehicle floor — on the many short residential/service edges
    // typical of a real OSM network) a headway gate compounds into
    // near-total gridlock even at low demand (see Palma validation run:
    // natural arrivals collapsed from 133k to 4.4k agents at demand
    // scale=0.4). Storage-based spillback alone already caps effective
    // throughput realistically without that extra, redundant constraint.
    if (next != kInvalidEdge && next < graph_.num_edges()) {
        const Cell& nc = cells_[next];
        float receiving = nc.storage_veh - static_cast<float>(nc.occupancy);
        if (receiving <= 0.0f)
            return false;
    }

    if (c.occupancy > 0) --c.occupancy;
    states_[e].occupancy.store(static_cast<float>(c.occupancy), std::memory_order_relaxed);
    states_[e].outflow_rate.store(c.capacity_veh_s, std::memory_order_relaxed);
    return true;
}

void LtmTrafficModel::force_remove(EdgeId e, AgentId /*a*/) {
    if (e >= graph_.num_edges()) return;
    Cell& c = cells_[e];
    if (c.occupancy > 0) --c.occupancy;
    states_[e].occupancy.store(static_cast<float>(c.occupancy), std::memory_order_relaxed);
    states_[e].travel_time_s.store(travel_time_for(e), std::memory_order_relaxed);
}

void LtmTrafficModel::update(SimTime /*t*/) {
    // Scan only active edges — O(active) not O(E). Recompute travel_time_s
    // from current occupancy so snapshots reflect the actual state even
    // when no agent has entered/exited since the last window.
    uint32_t out = 0;
    for (uint32_t i = 0; i < static_cast<uint32_t>(active_list_.size()); ++i) {
        EdgeId e   = active_list_[i];
        float  occ = static_cast<float>(cells_[e].occupancy);
        states_[e].occupancy.store(occ, std::memory_order_relaxed);
        float ff = graph_.free_flow_time(e);
        float tt;
        if (occ <= 0.0f) {
            tt = ff;
            is_active_[e] = 0;
        } else {
            tt = travel_time_for(e);
            active_list_[out++] = e;
        }
        states_[e].travel_time_s.store(tt, std::memory_order_relaxed);

        // Smooth tt/ff into the EMA — see LinkState::congestion_ema doc.
        if (ff > 0.0f) {
            float ratio = tt / ff;
            float ema   = states_[e].congestion_ema.load(std::memory_order_relaxed);
            states_[e].congestion_ema.store(
                ema + kCongestionEmaAlpha * (ratio - ema), std::memory_order_relaxed);
        }
    }
    active_list_.resize(out);
}

float LtmTrafficModel::current_travel_time(EdgeId e) const {
    if (e >= graph_.num_edges()) return 1.0f;
    return states_[e].travel_time_s.load(std::memory_order_relaxed);
}

std::span<const LinkState> LtmTrafficModel::link_states() const {
    return {states_.data(), states_.size()};
}

} // namespace nomad
