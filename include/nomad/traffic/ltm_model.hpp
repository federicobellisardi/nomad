#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

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

        // Node priority (opt-in, 0 = off). Each link has one major approach:
        // on a roundabout link the circulating roundabout approach, otherwise
        // the best road class, ties broken by the straightest continuation.
        // Vehicles from any other approach, and departures onto the link,
        // enter only while the link keeps more than priority_reserve_veh free
        // storage. Without it side entries took every freed slot and a ring
        // of links (dual carriageway + U-turns, Palma) reached jam density and
        // deadlocked. The stuck-time squeeze still overrides the block.
        float priority_reserve_veh = 0.0f;
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
    bool    has_capacity_departure(EdgeId e, EdgeId next) const       override;
    // True if `e` is the major approach of `next` (always true when the
    // priority rule is off).
    bool    is_major(EdgeId e, EdgeId next) const;
    bool    has_capacity_for(EdgeId e, EdgeId next) const               override;
    bool    on_exit_to  (EdgeId e, EdgeId next, EdgeId next_after,
                         AgentId a, SimTime t)                            override;
    void    assign_movement(EdgeId e, AgentId a, EdgeId next)            override;

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
    };

    float travel_time_for(EdgeId e) const;

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
    std::vector<float>    lanes_;
    std::vector<uint32_t> mv_off_;
    std::vector<int32_t>  mv_occ_;
    std::vector<uint32_t> agent_slot_;   // per agent, kNoSlot if none

    std::vector<uint8_t> is_active_;
    std::vector<EdgeId>  active_list_;
};

} // namespace nomad
