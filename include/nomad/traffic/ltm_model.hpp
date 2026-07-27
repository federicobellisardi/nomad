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
// Discharge-gate wait signal: travel_time_for()'s BPR term is driven by
// occupancy/storage_veh, which the departure/exit-link capacity gates keep
// permanently near 1.0 -- so it cannot reflect discharge-rate-limited delay
// once those gates are working correctly (occupancy simply never builds up
// enough to move the ratio). Cell::gate_wait_s tracks how long the discharge
// check has been continuously blocking exits and is added on top of the BPR
// term (only when enable_discharge_cap is true), giving congestion_ema and
// current_travel_time() -- and therefore schedule_reroutes() -- a real
// signal for this specific bottleneck.
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
    };

    float travel_time_for(EdgeId e) const;

    const Graph&        graph_;
    Config               cfg_;
    std::vector<Cell>    cells_;
    std::vector<LinkState> states_;

    // Dirty active-edge set — same pattern as QueueTrafficModel::update().
    std::vector<uint8_t> is_active_;
    std::vector<EdgeId>  active_list_;
};

} // namespace nomad
