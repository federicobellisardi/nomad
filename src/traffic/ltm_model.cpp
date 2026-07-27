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
        if (cfg_.enable_discharge_cap) {
            // Bucket starts FULL: an idle/lightly-used link must never be
            // artificially throttled from the very first exit.
            cells_[e].discharge_credit = std::max(
                1.0f, cells_[e].capacity_veh_s * cfg_.discharge_burst_s);
        }
        states_[e].travel_time_s.store(graph.free_flow_time(e), std::memory_order_relaxed);
    }
}

float LtmTrafficModel::travel_time_for(EdgeId e) const {
    const Cell& c = cells_[e];
    float ff  = graph_.free_flow_time(e);
    float cap = c.storage_veh;
    float occ = static_cast<float>(c.occupancy);
    // Discharge-gate wait: additive, only when the cap is enabled. Added
    // before the early-return below so a discharge-blocked edge still gets a
    // congestion signal even while occ/cap sits under 0.8 (the gate can bind
    // on a link that isn't storage-congested at all).
    float gate_term = cfg_.enable_discharge_cap ? c.gate_wait_s : 0.0f;
    if (cap <= 0.0f || occ / cap < 0.8f) return ff + gate_term;
    // Same BPR volume-delay shape as QueueTrafficModel, capped at vc=2 → 3.4× ff.
    // Reusing it here isolates the effect under test to the new exit-side
    // spillback gate rather than also changing the delay formula.
    float vc = std::min(occ / cap, 2.0f);
    return ff * (1.0f + 0.15f * std::pow(vc, 4.0f)) + gate_term;
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

bool LtmTrafficModel::has_capacity(EdgeId e) const {
    if (e >= graph_.num_edges()) return true;
    const Cell& c = cells_[e];
    return c.storage_veh - static_cast<float>(c.occupancy) > 0.0f;
}

bool LtmTrafficModel::on_exit(EdgeId e, EdgeId next, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return true;
    Cell& c = cells_[e];

    // Discharge-rate token bucket (opt-in, see class-level comment in the
    // header for why this is a bucket and not a rigid headway). Refilled
    // UNCONDITIONALLY here, before either gate below is checked, so a link
    // stalled by spillback on `next` isn't ALSO punished with a starved
    // bucket once spillback clears -- the two gates are independent.
    if (cfg_.enable_discharge_cap) {
        double dt = std::max(0.0, t - c.last_credit_update);
        float max_credit = std::max(1.0f, c.capacity_veh_s * cfg_.discharge_burst_s);
        c.discharge_credit = std::min(
            max_credit, c.discharge_credit + c.capacity_veh_s * static_cast<float>(dt));
        c.last_credit_update = t;
    }

    // Receiving-side spillback: block if the downstream link has no spare
    // storage. This is the behaviour QueueTrafficModel does not have — it
    // lets congestion genuinely propagate backward instead of only raising
    // the current link's own travel time.
    if (next != kInvalidEdge && next < graph_.num_edges() && !has_capacity(next))
        return false;

    // Sending-side discharge-rate cap (opt-in). A rigid per-exit headway
    // here was tried once and caused near-total gridlock (see header
    // comment) -- this bucket only blocks once a SUSTAINED burst exceeds
    // discharge_burst_s worth of capacity, not on isolated exits.
    if (cfg_.enable_discharge_cap) {
        if (c.discharge_credit < 1.0f) {
            // Gate is binding: start (or continue) tracking how long it's
            // been continuously blocked -- see travel_time_for().
            if (c.gate_block_since < 0.0) c.gate_block_since = t;
            c.gate_wait_s = static_cast<float>(t - c.gate_block_since);
            return false;
        }
        c.discharge_credit -= 1.0f;
        // Gate no longer binding for this exit -- reset.
        c.gate_block_since = -1.0;
        c.gate_wait_s = 0.0f;
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
