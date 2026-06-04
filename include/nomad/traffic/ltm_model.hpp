#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <vector>

namespace nomad {

// ── Link Transmission Model (Yperman 2007) ────────────────────────────────────
// Provides the exact solution to the LWR (Lighthill-Whitham-Richards) PDE on
// each link using cumulative vehicle counts N(x,t).
//
// Each link maintains two boundary counters:
//   N_A(t) — cumulative count at the upstream end (inflow)
//   N_B(t) — cumulative count at the downstream end (outflow)
//
// The LTM demand-supply framework:
//   demand(link)  = min(N_A(t) - N_B(t - τ), C × Δt)
//   supply(next)  = min(kj × L - (N_A(t-τ') - N_B(t)), C × Δt)
//   flow          = min(demand, supply)
//
// where τ = free-flow travel time, τ' = backward wave travel time, C = capacity,
// kj = jam density, L = link length.
//
// This model is analytically superior to the queue model for studies of
// shockwave propagation and queue length estimation, at the cost of requiring
// a fixed time step (time_step_s). Recommended for corridor-level studies.
// For metro-scale simulation with 1M agents, prefer QueueTrafficModel.
class LtmTrafficModel final : public ITrafficModel {
public:
    struct Config {
        float time_step_s     = 5.0f;   // simulation time step [s]
        float jam_density_vpm = 0.133f; // veh/m (= 1/7.5 m jam spacing)
    };

    explicit LtmTrafficModel(const Graph& graph);
    LtmTrafficModel(const Graph& graph, Config cfg);

    SimTime on_enter(EdgeId e, AgentId a, SimTime t) override;
    bool    on_exit (EdgeId e, AgentId a, SimTime t) override;
    void    update  (SimTime t)                      override;
    float   current_travel_time(EdgeId e) const      override;
    std::span<const LinkState> link_states()  const  override;
    std::string_view model_name()            const   override { return "ltm"; }

private:
    struct LtmLink {
        double N_upstream;    // cumulative count at upstream boundary
        double N_downstream;  // cumulative count at downstream boundary
        float  free_flow_tt;  // free-flow travel time τ [s]
        float  back_wave_tt;  // backward congestion wave travel time τ' [s]
        float  capacity_ps;   // capacity [veh/s]
        float  kj;            // jam density [veh/m]
        float  length_m;
    };

    float compute_demand (const LtmLink& l, SimTime t) const;
    float compute_supply (const LtmLink& l, SimTime t) const;

    const Graph&         graph_;
    Config               cfg_;
    std::vector<LtmLink> ltm_;
    std::vector<LinkState> states_;
    SimTime              last_update_{0.0};
};

} // namespace nomad
