#include <nomad/traffic/ltm_model.hpp>

#include <algorithm>
#include <limits>
#include <queue>
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

    if (cfg_.priority_reserve_veh > 0.0f || cfg_.priority_service) {
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
        // Short one-way cycles = unmapped roundabouts (priority_cycle_max_m).
        auto car_ok = [&](EdgeId x) {
            return edge_accessible(graph.edges[x].road_class, graph.edges[x].flags, AgentMode::Car);
        };
        std::vector<uint8_t> oneway(E, 0), circ(E, 0);
        for (uint32_t x = 0; x < E; ++x) {
            if (!car_ok(x) || src[x] == kInvalidNode || graph.edges[x].target >= N) continue;
            // major roads only: one-way residential / service / unclassified
            // blocks are street grids, not roundabouts
            if (graph.edges[x].road_class > static_cast<uint8_t>(RoadClass::TertiaryLink)) continue;
            bool twin = false;
            for (EdgeId y : graph.out_edges(graph.edges[x].target))
                if (graph.edges[y].target == src[x] && car_ok(y)) { twin = true; break; }
            oneway[x] = !twin;
        }
        if (cfg_.priority_cycle_max_m > 0.0f) {
            const float L = cfg_.priority_cycle_max_m;
            const auto signal = static_cast<uint8_t>(IntersectionType::TrafficSignal);
            std::vector<float>  dist(N, std::numeric_limits<float>::infinity());
            std::vector<EdgeId> pred(N, kInvalidEdge);
            std::vector<NodeId> touched;
            using QE = std::pair<float, NodeId>;
            for (uint32_t x = 0; x < E; ++x) {
                if (!oneway[x] || circ[x]) continue;
                const NodeId u = src[x], v = graph.edges[x].target;
                if (graph.nodes[u].intersection_type == signal || graph.nodes[v].intersection_type == signal) continue;
                const float budget = L - graph.edges[x].length_m;
                if (budget <= 0.0f) continue;
                // bounded Dijkstra v -> u over one-way, unsignalised car links
                std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
                dist[v] = 0.0f; touched.push_back(v); pq.push({0.0f, v});
                bool found = false;
                while (!pq.empty()) {
                    auto [d, a] = pq.top(); pq.pop();
                    if (d > dist[a]) continue;
                    if (a == u) { found = true; break; }
                    for (EdgeId y : graph.out_edges(a)) {
                        if (!oneway[y]) continue;
                        const NodeId b = graph.edges[y].target;
                        if (b != u && graph.nodes[b].intersection_type == signal) continue;
                        const float nd = d + graph.edges[y].length_m;
                        if (nd > budget || nd >= dist[b]) continue;
                        if (dist[b] == std::numeric_limits<float>::infinity()) touched.push_back(b);
                        dist[b] = nd; pred[b] = y; pq.push({nd, b});
                    }
                }
                if (found) {
                    circ[x] = 1;
                    for (NodeId a = u; a != v; a = src[pred[a]]) circ[pred[a]] = 1;
                }
                for (NodeId a : touched) { dist[a] = std::numeric_limits<float>::infinity(); pred[a] = kInvalidEdge; }
                touched.clear();
            }
            for (uint32_t x = 0; x < E; ++x) n_circ_ += circ[x];
        }
        auto circulating = [&](EdgeId x) {
            return (graph.edges[x].flags & EdgeFlags::Roundabout) || circ[x];
        };
        major_in_.assign(E, kInvalidEdge);
        for (uint32_t n = 0; n < E; ++n) {
            const NodeId u = src[n];
            if (u == kInvalidNode) continue;
            const NodeId w = graph.edges[n].target;
            const bool n_rb = circulating(n);
            const float hn = heading(u, w);
            EdgeId best = kInvalidEdge;
            int best_rank = 1 << 30; float best_turn = 10.0f;
            for (uint32_t k = in_ptr[u]; k < in_ptr[u + 1]; ++k) {
                const EdgeId e = in_idx[k];
                if (src[e] == w) continue;                   // U-turn of the same segment
                if (!edge_accessible(graph.edges[e].road_class, graph.edges[e].flags, AgentMode::Car)) continue;
                const bool e_rb = circulating(e);
                // rank: circulating roundabout approach first on a roundabout link,
                // then through roads before *_link connectors (ramps yield), then class
                const uint8_t rc = graph.edges[e].road_class;
                const bool is_link = rc == static_cast<uint8_t>(RoadClass::MotorwayLink) ||
                                     rc == static_cast<uint8_t>(RoadClass::TrunkLink) ||
                                     rc == static_cast<uint8_t>(RoadClass::PrimaryLink) ||
                                     rc == static_cast<uint8_t>(RoadClass::SecondaryLink) ||
                                     rc == static_cast<uint8_t>(RoadClass::TertiaryLink);
                const int rank = (n_rb && e_rb ? 0 : 1000) + (is_link ? 100 : 0) + rc;
                float turn = std::fabs(hn - heading(src[e], u));
                if (turn > 3.14159265f) turn = 2.0f * 3.14159265f - turn;
                if (rank < best_rank || (rank == best_rank && turn < best_turn)) {
                    best = e; best_rank = rank; best_turn = turn;
                }
            }
            major_in_[n] = best;
        }
        major_at_node_.assign(E, 0);
        for (uint32_t n = 0; n < E; ++n)
            if (major_in_[n] != kInvalidEdge) major_at_node_[major_in_[n]] = 1;
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
            const uint32_t N = graph.num_nodes();
            car_outdeg_.assign(N, 0);
            for (uint32_t u = 0; u < N; ++u) {
                int d = 0;
                for (EdgeId x : graph.out_edges(u))
                    if (edge_accessible(graph.edges[x].road_class, graph.edges[x].flags, AgentMode::Car)) ++d;
                car_outdeg_[u] = static_cast<uint8_t>(std::min(d, 2));
            }
        }
    }
}

LtmTrafficModel::Group LtmTrafficModel::lane_group(EdgeId e, std::span<const EdgeId> after) const {
    Group g;
    if (car_outdeg_.empty() || e >= graph_.num_edges()) return g;
    NodeId u = graph_.edges[e].target;
    float minl = 1e9f;
    for (EdgeId x : after) {
        if (x >= graph_.num_edges() || u >= car_outdeg_.size()) return Group{};
        minl = std::min(minl, lanes_[x]);
        if (car_outdeg_[u] > 1) { g.branch = x; g.lanes = minl; return g; }
        u = graph_.edges[x].target;
    }
    return Group{};
}

bool LtmTrafficModel::group_has_room(EdgeId e, const Group& g) const {
    if (g.branch == kInvalidEdge) return true;
    auto it = grp_occ_.find(group_key(e, g.branch));
    const int32_t occ = it == grp_occ_.end() ? 0 : it->second;
    const float share = std::min(1.0f, g.lanes / lanes_[e]);
    return static_cast<float>(occ) < std::max(1.0f, cells_[e].storage_veh * share);
}

bool LtmTrafficModel::has_capacity_path(EdgeId e, std::span<const EdgeId> after) const {
    if (!has_capacity_for(e, at(after, 0))) return false;
    if (!fifo_lookahead()) return true;
    return group_has_room(e, lane_group(e, after));
}

bool LtmTrafficModel::has_capacity_departure_path(EdgeId e, std::span<const EdgeId> after) const {
    if (!has_capacity_path(e, after)) return false;
    if (cfg_.priority_reserve_veh <= 0.0f || major_in_.empty() || e >= graph_.num_edges()) return true;
    const Cell& c = cells_[e];
    return c.storage_veh - static_cast<float>(c.occupancy) > cfg_.priority_reserve_veh;
}

bool LtmTrafficModel::on_exit_path(EdgeId e, std::span<const EdgeId> after, AgentId a, SimTime t) {
    const EdgeId next = at(after, 0);
    // spillback on `next`, for the agent's movement / lane group there
    const bool room = next == kInvalidEdge || next >= graph_.num_edges() ||
                      has_capacity_path(next, after.subspan(1));
    return on_exit_gate(e, next, room, a, t);
}

void LtmTrafficModel::assign_movement_path(EdgeId e, AgentId a, std::span<const EdgeId> after) {
    assign_movement(e, a, at(after, 0));
    if (!fifo_lookahead()) return;
    const Group g = lane_group(e, after);
    if (g.branch == kInvalidEdge) return;
    if (a >= agent_grp_.size())
        agent_grp_.resize(std::max<std::size_t>(a + 1, agent_grp_.size() * 2), ~uint64_t{0});
    const uint64_t k = group_key(e, g.branch);
    ++grp_occ_[k];
    agent_grp_[a] = k;
}

EdgeId LtmTrafficModel::fifo_group(EdgeId e, std::span<const EdgeId> after) const {
    if (!fifo_lookahead()) return at(after, 0);
    const Group g = lane_group(e, after);
    return g.branch == kInvalidEdge ? at(after, 0) : g.branch;
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
    if (a < agent_grp_.size() && agent_grp_[a] != ~uint64_t{0}) {
        auto it = grp_occ_.find(agent_grp_[a]);
        if (it != grp_occ_.end() && --it->second <= 0) grp_occ_.erase(it);
        agent_grp_[a] = ~uint64_t{0};
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
    release_movement(a);
    uint32_t s = movement_slot(e, next);
    if (s == kNoSlot) return;
    ++mv_occ_[s];
    agent_slot_[a] = s;
    (void)next_after;
}

bool LtmTrafficModel::has_capacity_for(EdgeId e, EdgeId next, EdgeId next_after) const {
    if (!has_capacity(e)) return false;
    if (!cfg_.movement_storage) return true;
    uint32_t s = movement_slot(e, next);
    if (s == kNoSlot) return true;
    (void)next_after;
    return static_cast<float>(mv_occ_[s]) < movement_cap(e, next);
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

bool LtmTrafficModel::is_major_at_node(EdgeId e) const {
    if (major_at_node_.empty() || e >= major_at_node_.size()) return true;
    return major_at_node_[e] != 0;
}

bool LtmTrafficModel::is_major(EdgeId e, EdgeId next) const {
    if (major_in_.empty() || next >= major_in_.size()) return true;
    return major_in_[next] == e;
}

bool LtmTrafficModel::has_capacity_departure(EdgeId e, EdgeId next, EdgeId next_after) const {
    if (!has_capacity_for(e, next, next_after)) return false;
    if (cfg_.priority_reserve_veh <= 0.0f || major_in_.empty() || e >= graph_.num_edges()) return true;
    const Cell& c = cells_[e];
    return c.storage_veh - static_cast<float>(c.occupancy) > cfg_.priority_reserve_veh;
}

bool LtmTrafficModel::on_exit(EdgeId e, EdgeId next, AgentId a, SimTime t) {
    return on_exit_to(e, next, kInvalidEdge, a, t);
}

bool LtmTrafficModel::on_exit_to(EdgeId e, EdgeId next, EdgeId next_after,
                                 AgentId a, SimTime t, EdgeId next_after2) {
    (void)next_after2;
    const bool room = next == kInvalidEdge || next >= graph_.num_edges() ||
                      has_capacity_for(next, next_after);
    return on_exit_gate(e, next, room, a, t);
}

bool LtmTrafficModel::on_exit_gate(EdgeId e, EdgeId next, bool room, AgentId a, SimTime t) {
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
    bool spillback_blocked = !room;
    // Node priority: a minor approach keeps a free-storage reserve on `next`.
    if (!spillback_blocked && cfg_.priority_reserve_veh > 0.0f && !major_in_.empty() && next != kInvalidEdge &&
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
         t - cells_[next].last_squeeze_in >= cfg_.stuck_time_s) &&
        (!cfg_.squeeze_stuck_receiver || next >= graph_.num_edges() ||
         t - cells_[next].last_exit >= cfg_.stuck_time_s) &&
        (!cfg_.minor_squeeze_stuck_receiver || major_in_.empty() || next >= graph_.num_edges() ||
         major_in_[next] == e || t - cells_[next].last_exit >= cfg_.stuck_time_s)) {
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
    c.last_exit = t;

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
