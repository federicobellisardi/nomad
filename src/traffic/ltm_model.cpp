#include <nomad/traffic/ltm_model.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace nomad {

namespace {
float lanes_of(const Graph& g, EdgeId e) {
    if (g.has_lanes()) return std::max(1.0f, g.edge_lanes[e]);
    return std::max(1.0f, g.edges[e].capacity / 1600.0f);
}
} // namespace

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
        // Lanes: from the graph (v2, OSM per direction) when available;
        // otherwise the legacy estimate capacity / 1600, which mixes in the
        // per-class lane capacity and the intersection derating.
        float estimated_lanes = lanes_of(graph, e);
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

    if (cfg_.priority_reserve_veh > 0.0f) {
        // in-edges per node
        const uint32_t N = graph.num_nodes();
        std::vector<uint32_t> in_ptr(N + 1, 0);
        for (uint32_t e = 0; e < E; ++e)
            if (graph.edges[e].target < N) ++in_ptr[graph.edges[e].target + 1];
        for (uint32_t u = 0; u < N; ++u) in_ptr[u + 1] += in_ptr[u];
        std::vector<EdgeId> in_idx(E);
        std::vector<NodeId> src(E, kInvalidNode);
        {
            std::vector<uint32_t> fill = in_ptr;
            for (uint32_t u = 0; u < N; ++u)
                for (EdgeId e : graph.out_edges(u)) {
                    src[e] = u;
                    NodeId t = graph.edges[e].target;
                    if (t < N) in_idx[fill[t]++] = e;
                }
        }
        auto heading = [&](NodeId a, NodeId b) {
            const float dx = (graph.nodes[b].lon - graph.nodes[a].lon) *
                             std::cos(graph.nodes[a].lat * 3.14159265f / 180.0f);
            const float dy = graph.nodes[b].lat - graph.nodes[a].lat;
            return std::atan2(dy, dx);
        };
        major_in_.assign(E, kInvalidEdge);
        for (uint32_t n = 0; n < E; ++n) {
            const NodeId u = src[n];
            if (u == kInvalidNode) continue;
            const NodeId w = graph.edges[n].target;
            const bool n_rb = graph.edges[n].flags & EdgeFlags::Roundabout;
            const float hn = heading(u, w);
            EdgeId best = kInvalidEdge;
            int best_rank = 1 << 30; float best_turn = 10.0f;
            for (uint32_t k = in_ptr[u]; k < in_ptr[u + 1]; ++k) {
                const EdgeId e = in_idx[k];
                if (src[e] == w) continue;                   // U-turn of the same segment
                if (!edge_accessible(graph.edges[e].road_class, graph.edges[e].flags, AgentMode::Car)) continue;
                const bool e_rb = graph.edges[e].flags & EdgeFlags::Roundabout;
                // rank: circulating roundabout approach first on a roundabout link, then class
                const int rank = (n_rb && e_rb ? 0 : 1000) + graph.edges[e].road_class;
                float turn = std::fabs(hn - heading(src[e], u));
                if (turn > 3.14159265f) turn = 2.0f * 3.14159265f - turn;
                if (rank < best_rank || (rank == best_rank && turn < best_turn)) {
                    best = e; best_rank = rank; best_turn = turn;
                }
            }
            major_in_[n] = best;
        }
    }

    if (cfg_.movement_storage) {
        lanes_.resize(E);
        mv_off_.resize(E + 1);
        uint32_t off = 0;
        for (uint32_t e = 0; e < E; ++e) {
            lanes_[e]  = lanes_of(graph, e);
            mv_off_[e] = off;
            NodeId u = graph.edges[e].target;
            if (u < graph.num_nodes()) off += graph.row_ptr[u + 1] - graph.row_ptr[u];
        }
        mv_off_[E] = off;
        mv_occ_.assign(off, 0);
        if (cfg_.movement_lookahead) {
            // one group counter per (e, next, next_after)
            mv2_off_.assign(off + 1, 0);
            uint32_t off2 = 0;
            for (uint32_t e = 0; e < E; ++e) {
                NodeId u = graph.edges[e].target;
                if (u >= graph.num_nodes()) continue;
                for (uint32_t k = graph.row_ptr[u]; k < graph.row_ptr[u + 1]; ++k) {
                    mv2_off_[mv_off_[e] + (k - graph.row_ptr[u])] = off2;
                    NodeId w = graph.edges[graph.col_idx[k]].target;
                    if (w < graph.num_nodes()) off2 += graph.row_ptr[w + 1] - graph.row_ptr[w];
                }
            }
            mv2_off_[off] = off2;
            mv2_occ_.assign(off2, 0);
        }
    }
}

uint32_t LtmTrafficModel::group_slot(EdgeId e, EdgeId next, EdgeId next_after) const {
    if (mv2_off_.empty() || next_after == kInvalidEdge || next >= graph_.num_edges()) return kNoSlot;
    const uint32_t s = movement_slot(e, next);
    if (s == kNoSlot) return kNoSlot;
    NodeId w = graph_.edges[next].target;
    if (w >= graph_.num_nodes()) return kNoSlot;
    const uint32_t b = graph_.row_ptr[w], end = graph_.row_ptr[w + 1];
    for (uint32_t k = b; k < end; ++k)
        if (graph_.col_idx[k] == next_after) return mv2_off_[s] + (k - b);
    return kNoSlot;
}

uint32_t LtmTrafficModel::movement_slot(EdgeId e, EdgeId next) const {
    if (e >= graph_.num_edges() || next == kInvalidEdge) return kNoSlot;
    NodeId u = graph_.edges[e].target;
    if (u >= graph_.num_nodes()) return kNoSlot;
    const uint32_t b = graph_.row_ptr[u], end = graph_.row_ptr[u + 1];
    for (uint32_t k = b; k < end; ++k)
        if (graph_.col_idx[k] == next) return mv_off_[e] + (k - b);
    return kNoSlot;
}

float LtmTrafficModel::movement_cap(EdgeId e, EdgeId next) const {
    float share = std::min(1.0f, lanes_[next] / lanes_[e]);
    return std::max(1.0f, cells_[e].storage_veh * share);
}

void LtmTrafficModel::release_movement(AgentId a) {
    if (a < agent_slot2_.size() && agent_slot2_[a] != kNoSlot) {
        if (mv2_occ_[agent_slot2_[a]] > 0) --mv2_occ_[agent_slot2_[a]];
        agent_slot2_[a] = kNoSlot;
    }
    if (a >= agent_slot_.size()) return;
    uint32_t s = agent_slot_[a];
    if (s == kNoSlot) return;
    if (mv_occ_[s] > 0) --mv_occ_[s];
    agent_slot_[a] = kNoSlot;
}

void LtmTrafficModel::assign_movement(EdgeId e, AgentId a, EdgeId next, EdgeId next_after) {
    if (!cfg_.movement_storage) return;
    if (a >= agent_slot_.size())
        agent_slot_.resize(std::max<std::size_t>(a + 1, agent_slot_.size() * 2), kNoSlot);
    if (!mv2_off_.empty() && a >= agent_slot2_.size())
        agent_slot2_.resize(std::max<std::size_t>(a + 1, agent_slot2_.size() * 2), kNoSlot);
    release_movement(a);
    uint32_t s = movement_slot(e, next);
    if (s == kNoSlot) return;
    ++mv_occ_[s];
    agent_slot_[a] = s;
    uint32_t g = group_slot(e, next, next_after);
    if (g == kNoSlot) return;
    ++mv2_occ_[g];
    agent_slot2_[a] = g;
}

bool LtmTrafficModel::has_capacity_for(EdgeId e, EdgeId next, EdgeId next_after) const {
    if (!has_capacity(e)) return false;
    if (!cfg_.movement_storage) return true;
    uint32_t s = movement_slot(e, next);
    if (s == kNoSlot) return true;
    if (static_cast<float>(mv_occ_[s]) >= movement_cap(e, next)) return false;
    uint32_t g = group_slot(e, next, next_after);
    if (g == kNoSlot) return true;
    const float share = std::min(1.0f, std::min(lanes_[next], lanes_[next_after]) / lanes_[e]);
    return static_cast<float>(mv2_occ_[g]) < std::max(1.0f, cells_[e].storage_veh * share);
}

float LtmTrafficModel::travel_time_for(EdgeId e) const {
    const Cell& c = cells_[e];
    float ff  = graph_.free_flow_time(e);
    float cap = c.storage_veh;
    float occ = static_cast<float>(c.occupancy);
    // Exit-gate wait: additive, always on (see header comment -- spillback
    // blocking is the model's default behaviour, not opt-in). Added before
    // the early-return below so a blocked edge still gets a congestion
    // signal even while occ/cap sits under 0.8 (the gate can bind on a link
    // that isn't storage-congested at all, e.g. a short link stuck behind a
    // saturated downstream one).
    float gate_term = c.gate_wait_s;
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
    // With movement storage, the exit-gate wait of whatever movement is
    // blocked at the head of the link must not delay vehicles heading to a
    // free movement: their scheduled exit uses the link travel time without
    // the gate term. Vehicles of the blocked movement still wait, through
    // the 1 s exit retries. The published travel_time_s keeps the gate term
    // as the congestion signal for rerouting.
    if (cfg_.movement_storage) return t + (tt - c.gate_wait_s);
    return t + tt;
}

bool LtmTrafficModel::has_capacity(EdgeId e) const {
    if (e >= graph_.num_edges()) return true;
    const Cell& c = cells_[e];
    return c.storage_veh - static_cast<float>(c.occupancy) > 0.0f;
}

bool LtmTrafficModel::is_major(EdgeId e, EdgeId next) const {
    if (major_in_.empty() || next >= major_in_.size()) return true;
    return major_in_[next] == e;
}

bool LtmTrafficModel::has_capacity_departure(EdgeId e, EdgeId next, EdgeId next_after) const {
    if (!has_capacity_for(e, next, next_after)) return false;
    if (major_in_.empty() || e >= graph_.num_edges()) return true;
    const Cell& c = cells_[e];
    return c.storage_veh - static_cast<float>(c.occupancy) > cfg_.priority_reserve_veh;
}

bool LtmTrafficModel::on_exit(EdgeId e, EdgeId next, AgentId a, SimTime t) {
    return on_exit_to(e, next, kInvalidEdge, a, t);
}

bool LtmTrafficModel::on_exit_to(EdgeId e, EdgeId next, EdgeId next_after,
                                 AgentId a, SimTime t, EdgeId next_after2) {
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
    // the current link's own travel time. Always active, not opt-in.
    bool spillback_blocked =
        next != kInvalidEdge && next < graph_.num_edges() && !has_capacity_for(next, next_after, next_after2);
    // Node priority: a minor approach keeps a free-storage reserve on `next`.
    if (!spillback_blocked && !major_in_.empty() && next != kInvalidEdge &&
        next < graph_.num_edges() && major_in_[next] != e) {
        const Cell& r = cells_[next];
        if (r.storage_veh - static_cast<float>(r.occupancy) <= cfg_.priority_reserve_veh)
            spillback_blocked = true;
    }

    // Sending-side discharge-rate cap (opt-in). A rigid per-exit headway
    // here was tried once and caused near-total gridlock (see header
    // comment) -- this bucket only blocks once a SUSTAINED burst exceeds
    // discharge_burst_s worth of capacity, not on isolated exits.
    bool discharge_blocked = cfg_.enable_discharge_cap && c.discharge_credit < 1.0f;

    // Stuck-time rule: a spillback block that has lasted stuck_time_s is
    // overridden (the downstream link goes over storage by one vehicle).
    if (spillback_blocked && !discharge_blocked && cfg_.stuck_time_s > 0.0f &&
        c.gate_block_since >= 0.0 && t - c.gate_block_since >= cfg_.stuck_time_s &&
        (!cfg_.squeeze_per_receiver ||
         t - cells_[next].last_squeeze_in >= cfg_.stuck_time_s)) {
        spillback_blocked = false;
        ++n_squeezed_;
        cells_[next].last_squeeze_in = t;
    }

    if (spillback_blocked || discharge_blocked) {
        // Gate is binding (from either cause): start (or continue) tracking
        // how long the head-of-queue exit has been continuously blocked --
        // see travel_time_for() and the header comment for why this is
        // unconditional rather than discharge-cap-only.
        if (c.gate_block_since < 0.0) c.gate_block_since = t;
        c.gate_wait_s = static_cast<float>(t - c.gate_block_since);
        return false;
    }

    if (cfg_.enable_discharge_cap) c.discharge_credit -= 1.0f;
    // Gate no longer binding for this exit -- reset.
    c.gate_block_since = -1.0;
    c.gate_wait_s = 0.0f;

    if (c.occupancy > 0) --c.occupancy;
    release_movement(a);
    states_[e].occupancy.store(static_cast<float>(c.occupancy), std::memory_order_relaxed);
    states_[e].outflow_rate.store(c.capacity_veh_s, std::memory_order_relaxed);
    return true;
}

void LtmTrafficModel::force_remove(EdgeId e, AgentId a) {
    if (e >= graph_.num_edges()) return;
    Cell& c = cells_[e];
    if (c.occupancy > 0) --c.occupancy;
    release_movement(a);
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
