#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <span>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── Cell-transmission-style discharge model ───────────────────────────────────
// One cell per edge (Daganzo 1994 CTM, single-cell-per-link variant). Unlike
// QueueTrafficModel (pure BPR volume-delay: raises travel time under load but
// never blocks entry), this model enforces a real receiving-side flow gate:
//
//   receiving(next) = storage_veh(next) - occupancy(next)
//
// on_exit(e, next, ...) blocks (returns false) whenever `next` has no spare
// storage — i.e. spillback. Congestion this way genuinely propagates
// backward onto upstream links and drains once downstream clears, instead of
// an agent sitting at a fixed BPR delay ceiling until it's teleported.
//
// Sending-side discharge-rate cap (opt-in, see Config::enable_discharge_cap):
// a token/credit bucket, NOT a rigid per-exit headway. A rigid headway
// (block any exit within 1/capacity_veh_s seconds of the last one) was tried
// once before on this same storage-spillback model and caused near-total
// gridlock even at low demand ("natural arrivals collapsed from 133k to 4.4k
// agents at demand scale=0.4") -- it blocks even isolated, non-sustained
// exits, inflating dwell time on short links, which then compounds backward
// through the (unrelated) upstream spillback check. A token bucket instead
// accrues discharge credit continuously at capacity_veh_s, capped at
// max(1.0, capacity_veh_s * discharge_burst_s), and only throttles once a
// SUSTAINED burst exceeds that budget -- an isolated exit on a lightly used
// link almost always finds the bucket full. Default OFF so existing
// spillback-only behaviour is unchanged unless explicitly enabled.
//
// Exit-gate wait signal: travel_time_for()'s BPR term is driven by
// occ/storage_veh, but occupancy can PHYSICALLY never exceed storage_veh
// (on_enter of the next link is refused once it's full) -- so vc = occ/cap
// saturates at ~1.0 and the BPR term is hard-capped at ff*1.15, no matter
// how long an edge has sat completely gridlocked at jam density. Confirmed
// empirically: a Palma corridor edge pinned at ~100% storage occupancy for
// 16+ continuous hours still reported travel_time_ratio ~1.15 the entire
// time, because the formula has no other way to express "still full."
//
// Cell::gate_wait_s tracks how long the HEAD-OF-QUEUE exit has been
// continuously blocked, by EITHER cause -- receiving-side spillback
// (always active, not opt-in: on_exit's `next` has no spare storage) or
// sending-side discharge-rate exhaustion (opt-in, enable_discharge_cap) --
// and is added on top of the BPR term unconditionally. What matters for the
// congestion signal is how long this link's traffic has failed to actually
// move, not which specific gate is currently binding; spillback is the
// model's default, always-on blocking behaviour, so the signal for it can't
// be opt-in either. This gives congestion_ema and current_travel_time() --
// and therefore schedule_reroutes() -- a real, unbounded-growth signal for
// genuine gridlock, instead of silently reporting near-free-flow forever.
//
// Note this only changes the REPORTED congestion signal (travel_time_for());
// on_exit()'s bool return and occupancy/spillback mechanics are unchanged.
//
// storage_veh and capacity accounting reuse the exact conventions validated
// in QueueTrafficModel (50m effective-length floor, kJamDensity = 1/7.5) so
// the two models are directly comparable.
class LtmTrafficModel final : public ITrafficModel {
public:
    struct Config {
        float jam_density_vpm = kJamDensity; // veh/m (= 1/7.5 m jam spacing)

        // Sending-side discharge-rate cap -- see class-level comment above.
        bool  enable_discharge_cap = false;
        // Burst window: max_credit = max(1.0, capacity_veh_s * discharge_burst_s).
        // Larger -> more tolerant of platoons before throttling kicks in;
        // smaller converges toward a rigid headway (avoid going too small).
        float discharge_burst_s   = 10.0f;

        // Movement-share storage (opt-in). Vehicles on link e heading to
        // next link n may hold at most storage(e) * min(1, lanes(n)/lanes(e))
        // (>= 1 vehicle), lanes = max(1, capacity/1600). Without it a queue
        // for a narrow movement (1-lane off-ramp) can fill a wide link and
        // block every other movement (diverge blocking), which collapsed the
        // Ma-20 in the Palma evening runs. Total storage still applies.
        bool  movement_storage    = false;

        // Movement lookahead (opt-in, needs movement_storage). Vehicles on e
        // are grouped into lane groups by the branch they take at the first
        // diverge downstream along their route (first node with more than one
        // car out-link; between e and it there is no other choice, so the
        // path is fixed). A group may hold at most storage(e) * min(1,
        // min lanes along the path up to and including the branch / lanes(e)),
        // and the QNode FIFO holds per group. Without it, a full off-ramp
        // whose diverge lies one or more links downstream (motorway chains of
        // micro-links) blocks every vehicle on e, including through traffic
        // (Palma: Ma-20 / Ma-19 queues discharging at 2-18% of capacity with
        // an empty mainline downstream).
        bool  movement_lookahead  = false;

        // Gridlock resolution (opt-in, 0 = off). When the exit of link e has
        // been continuously blocked by downstream spillback for at least
        // stuck_time_s, the head vehicle enters the next link anyway, over its
        // storage (same idea as MATSim's qsim stuckTime). The block timer
        // restarts after every exit, so a blocked link releases at most one
        // vehicle per stuck_time_s. The discharge-rate gate is never bypassed.
        float stuck_time_s        = 0.0f;

        // Per-receiver squeeze limit (opt-in, needs stuck_time_s > 0). A link
        // accepts at most one squeezed vehicle per stuck_time_s, whichever
        // upstream link sends it. Without it, k blocked upstream links each
        // squeeze one vehicle per stuck_time_s into the same receiver, which
        // itself drains at one per stuck_time_s when blocked: micro-links
        // piled up hundreds of vehicles over storage (Palma: ~18k excess veh).
        bool  squeeze_per_receiver = false;

        // Squeeze only into a stuck receiver (opt-in, needs stuck_time_s > 0).
        // The stuck-time rule may override a spillback block only if the
        // receiving link itself has had no exit for stuck_time_s (a true
        // deadlock). Without it, k blocked in-links each squeeze one vehicle
        // per stuck_time_s into a receiver that is still draining; when it
        // drains slower than k / stuck_time_s it never gets back below
        // storage, every in-link moves at exactly 1 / stuck_time_s whatever
        // its capacity, and the receiver overfills without bound (Palma:
        // Ma-19 N on-ramp chain at 30 veh / 15 min for eight hours, micro-
        // links at up to 80x storage). With it a draining receiver hands its
        // freed slots to the in-links by the node model (capacity-weighted).
        bool  squeeze_stuck_receiver = false;

        // Node priority (opt-in, 0 = off). Each link has one major approach:
        // on a roundabout link the circulating roundabout approach, otherwise
        // the best road class, ties broken by the straightest continuation.
        // Vehicles from any other approach, and departures onto the link,
        // enter only while the link keeps more than priority_reserve_veh free
        // storage. Without it side entries took every freed slot and a ring
        // of links (dual carriageway + U-turns, Palma) reached jam density and
        // deadlocked. The stuck-time squeeze still overrides the block.
        float priority_reserve_veh = 0.0f;
        // Priority-ordered node service (opt-in, QNode only). At every node
        // step the in-links that are the major approach of some out-link of
        // the node are served first; the others take the receiving capacity
        // that is left (give-way). No storage reserve is kept. Without it the
        // capacity-proportional draw lets entries compete with circulating
        // flow, so a ring of links (Palma, Ma-20 / Ma-1110 Son Espases)
        // fills and locks.
        // Major approach (both priority rules): the circulating approach on a
        // roundabout link, otherwise a through road over any *_link connector
        // (ramps yield), then the best road class, ties broken by the
        // straightest continuation.
        bool  priority_service = false;
        // Unmapped roundabouts (opt-in, 0 = off; used by both priority rules).
        // A one-way link of a major road class (motorway .. tertiary_link; no
        // car link in the opposite direction between the same nodes) that lies
        // on a directed cycle of such links of total length
        // <= priority_cycle_max_m, through no signalised node, counts as
        // circulating (one-way residential blocks are street grids, not
        // roundabouts: in Palma they would add ~6,300 links): like a tagged roundabout link it is the
        // major approach where the cycle continues. OSM often maps
        // roundabouts and dumbbell interchanges as plain one-way carriageways
        // joined by U-turn connectors (Palma, Ma-20 / Ma-1110 Son Espases),
        // where the straightest-continuation rule would give way to the
        // entries instead.
        float priority_cycle_max_m = 0.0f;
    };

    explicit LtmTrafficModel(const Graph& graph);
    LtmTrafficModel(const Graph& graph, Config cfg);

    SimTime on_enter    (EdgeId e, AgentId a, SimTime t)              override;
    bool    on_exit     (EdgeId e, EdgeId next, AgentId a, SimTime t) override;
    void    force_remove(EdgeId e, AgentId a)                         override;
    void    update      (SimTime t)                                   override;
    float   current_travel_time(EdgeId e) const                       override;
    std::span<const LinkState> link_states()  const                   override;
    std::string_view model_name()            const  override { return "ltm"; }
    bool    has_capacity(EdgeId e) const                              override;
    bool    has_capacity_departure(EdgeId e, EdgeId next,
                                   EdgeId next_after = kInvalidEdge) const override;
    // True if `e` is the major approach of `next` (always true when the
    // priority rule is off).
    bool    is_major(EdgeId e, EdgeId next) const;
    bool    priority_service() const override { return cfg_.priority_service; }
    // True if e is the major approach of at least one out-link of its target
    // node (always true when no priority rule is on).
    bool    is_major_at_node(EdgeId e) const override;
    // Number of links flagged as circulating by the short-cycle rule.
    std::size_t circulating_links() const { return n_circ_; }
    bool    has_capacity_for(EdgeId e, EdgeId next,
                             EdgeId next_after = kInvalidEdge) const      override;
    bool    on_exit_to  (EdgeId e, EdgeId next, EdgeId next_after,
                         AgentId a, SimTime t,
                         EdgeId next_after2 = kInvalidEdge)               override;
    void    assign_movement(EdgeId e, AgentId a, EdgeId next,
                            EdgeId next_after = kInvalidEdge)             override;
    bool    fifo_per_movement() const override { return cfg_.movement_storage; }
    bool    fifo_lookahead() const override {
        return cfg_.movement_storage && cfg_.movement_lookahead;
    }
    bool    has_capacity_path(EdgeId e, std::span<const EdgeId> after) const override;
    bool    has_capacity_departure_path(EdgeId e, std::span<const EdgeId> after) const override;
    bool    on_exit_path(EdgeId e, std::span<const EdgeId> after, AgentId a, SimTime t) override;
    void    assign_movement_path(EdgeId e, AgentId a, std::span<const EdgeId> after) override;
    EdgeId  fifo_group(EdgeId e, std::span<const EdgeId> after) const override;

    // Exits forced through a blocked spillback gate by the stuck_time_s rule.
    uint64_t squeezed_exits() const { return n_squeezed_; }

private:
    struct Cell {
        int32_t  occupancy{0};
        float    storage_veh{0.0f};     // max vehicles before jam density
        // Discharge capacity [veh/s]. Always written to LinkState::outflow_rate
        // for reporting; ALSO actually enforced (token-bucket refill rate)
        // when cfg_.enable_discharge_cap is true -- see on_exit().
        float    capacity_veh_s{0.0f};
        // Token-bucket state for the discharge-rate cap. Unused/inert when
        // enable_discharge_cap is false.
        float    discharge_credit{0.0f};    // accrued exit "tokens" [veh]
        double   last_credit_update{0.0};   // sim time of last accrual

        // Discharge-gate wait signal (opt-in, see travel_time_for()). Tracks
        // how long the head-of-queue exit has been continuously blocked by
        // the discharge-credit check -- an additive congestion proxy that,
        // unlike occupancy/storage_veh, is not saturated near 1.0 by the
        // departure/exit-link capacity gates. Unused/inert when
        // enable_discharge_cap is false. Sentinel is -1.0, not 0.0: SimTime
        // 0.0 is a legitimate, common simulation timestamp (sim start), so
        // it cannot double as "not currently blocked".
        double   gate_block_since{-1.0};    // sim time the block started; -1 = not blocked
        float    gate_wait_s{0.0f};         // duration of the current continuous block
        // Sim time of the last squeezed entry INTO this link (squeeze_per_receiver).
        double   last_squeeze_in{-1e18};
        // Sim time of the last exit FROM this link (squeeze_stuck_receiver).
        double   last_exit{-1e18};
    };

    float travel_time_for(EdgeId e) const;
    // Exit gates of e given whether `next` has room for the agent.
    bool  on_exit_gate(EdgeId e, EdgeId next, bool room, AgentId a, SimTime t);

    const Graph&        graph_;
    Config               cfg_;
    std::vector<Cell>    cells_;
    std::vector<LinkState> states_;
    uint64_t             n_squeezed_{0};

    // Dirty active-edge set — same pattern as QueueTrafficModel::update().
    // Movement-share storage (cfg_.movement_storage). For link e, the
    // movements are the out-edges of target(e), i.e. CSR positions
    // row_ptr[target(e)] .. row_ptr[target(e)+1); their occupancy counters
    // live at mv_occ_[mv_off_[e] + (pos - row_ptr[target(e)])].
    static constexpr uint32_t kNoSlot = 0xFFFFFFFFu;
    uint32_t movement_slot(EdgeId e, EdgeId next) const;
    float    movement_cap (EdgeId e, EdgeId next) const;
    void     release_movement(AgentId a);
    std::vector<EdgeId>   major_in_;   // major approach of each link (priority rule)
    std::vector<uint8_t>  major_at_node_;   // e is major for some out-link of target(e)
    std::size_t           n_circ_{0};       // links on short one-way cycles
    std::vector<float>    lanes_;
    std::vector<uint32_t> mv_off_;
    std::vector<int32_t>  mv_occ_;
    std::vector<uint32_t> agent_slot_;   // per agent, kNoSlot if none
    // Movement lookahead: lane group of a vehicle on e = branch at the first
    // diverge along `after`, with the min lanes up to it (kInvalidEdge: no
    // diverge before the trip ends, no group limit).
    struct Group { EdgeId branch = kInvalidEdge; float lanes = 0.0f; };
    Group lane_group(EdgeId e, std::span<const EdgeId> after) const;
    bool  group_has_room(EdgeId e, const Group& g) const;
    static uint64_t group_key(EdgeId e, EdgeId b) { return (uint64_t(e) << 32) | b; }
    std::vector<uint8_t>  car_outdeg_;              // per node, capped at 2
    std::unordered_map<uint64_t, int32_t> grp_occ_;
    std::vector<uint64_t> agent_grp_;               // per agent, ~0 if none

    std::vector<uint8_t> is_active_;
    std::vector<EdgeId>  active_list_;
};

} // namespace nomad
