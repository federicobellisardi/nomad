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
// No separate per-link discharge-headway gate is applied on the sending
// side: combined with per-link storage caps that are already small on short
// residential/service edges, a headway gate compounds into near-total
// gridlock even at low demand (see project history — this is the same
// failure mode previously diagnosed with QueueTrafficModel's headway
// option). Storage-based spillback alone already caps effective throughput.
//
// storage_veh and capacity accounting reuse the exact conventions validated
// in QueueTrafficModel (50m effective-length floor, kJamDensity = 1/7.5) so
// the two models are directly comparable.
class LtmTrafficModel final : public ITrafficModel {
public:
    struct Config {
        float jam_density_vpm = kJamDensity; // veh/m (= 1/7.5 m jam spacing)
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

private:
    struct Cell {
        int32_t  occupancy{0};
        float    storage_veh{0.0f};     // max vehicles before jam density
        float    capacity_veh_s{0.0f};  // discharge capacity [veh/s], informational (outflow_rate)
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
