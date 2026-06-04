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
    ltm_.resize(E);
    states_.resize(E);

    for (uint32_t e = 0; e < E; ++e) {
        const EdgeData& ed = graph.edges[e];
        float ff_tt  = graph.free_flow_time(e);
        float speed  = ed.free_flow_speed > 0.0f ? ed.free_flow_speed : 1.0f;
        float cap_ps = ed.capacity / 3600.0f;           // veh/s
        float kj     = cfg.jam_density_vpm;             // veh/m

        // Backward wave speed: w = capacity / (kj × length)
        float back_wave_speed = (kj * ed.length_m > 0)
            ? cap_ps / (kj * ed.length_m) : speed;

        ltm_[e].N_upstream     = 0.0;
        ltm_[e].N_downstream   = 0.0;
        ltm_[e].free_flow_tt   = ff_tt;
        ltm_[e].back_wave_tt   = ed.length_m / back_wave_speed;
        ltm_[e].capacity_ps    = cap_ps;
        ltm_[e].kj             = kj;
        ltm_[e].length_m       = ed.length_m;

        states_[e].occupancy    .store(0.0f);
        states_[e].inflow_rate  .store(0.0f);
        states_[e].outflow_rate .store(0.0f);
        states_[e].travel_time_s.store(ff_tt);
    }
}

float LtmTrafficModel::compute_demand(const LtmLink& l, SimTime /*t*/) const {
    // Demand = min(vehicles that can leave in Δt, capacity × Δt)
    double send = l.N_upstream - l.N_downstream;
    return static_cast<float>(
        std::min(send, static_cast<double>(l.capacity_ps * cfg_.time_step_s)));
}

float LtmTrafficModel::compute_supply(const LtmLink& l, SimTime /*t*/) const {
    // Supply = capacity × Δt if not congested, else shock wave constraint
    double receive = l.kj * l.length_m - (l.N_upstream - l.N_downstream);
    return static_cast<float>(
        std::min(static_cast<double>(l.capacity_ps * cfg_.time_step_s),
                  receive));
}

SimTime LtmTrafficModel::on_enter(EdgeId e, AgentId /*a*/, SimTime t) {
    assert(e < static_cast<EdgeId>(ltm_.size()));
    ltm_[e].N_upstream += 1.0;
    float occ = static_cast<float>(ltm_[e].N_upstream - ltm_[e].N_downstream);
    states_[e].occupancy.store(occ);
    return t + states_[e].travel_time_s.load();
}

bool LtmTrafficModel::on_exit(EdgeId e, AgentId /*a*/, SimTime /*t*/) {
    assert(e < static_cast<EdgeId>(ltm_.size()));
    float supply = compute_supply(ltm_[e], 0.0);
    if (supply <= 0.0f) return false;
    ltm_[e].N_downstream += 1.0;
    states_[e].occupancy.store(
        static_cast<float>(ltm_[e].N_upstream - ltm_[e].N_downstream));
    return true;
}

void LtmTrafficModel::update(SimTime t) {
    if (t - last_update_ < cfg_.time_step_s) return;
    last_update_ = t;

    const uint32_t E = graph_.num_edges();
    for (uint32_t e = 0; e < E; ++e) {
        LtmLink& l = ltm_[e];
        float occ  = static_cast<float>(l.N_upstream - l.N_downstream);
        float dens = l.length_m > 0.0f ? occ / l.length_m : 0.0f;
        float speed= (dens < l.kj * 0.8f)
            ? graph_.edges[e].free_flow_speed
            : graph_.edges[e].free_flow_speed * (1.0f - dens / l.kj);
        float tt   = speed > 0.0f ? l.length_m / speed : l.free_flow_tt * 10.0f;

        states_[e].occupancy    .store(occ);
        states_[e].travel_time_s.store(tt);
        states_[e].inflow_rate  .store(l.capacity_ps);
    }
}

float LtmTrafficModel::current_travel_time(EdgeId e) const {
    assert(e < static_cast<EdgeId>(ltm_.size()));
    return states_[e].travel_time_s.load();
}

std::span<const LinkState> LtmTrafficModel::link_states() const {
    return {states_.data(), states_.size()};
}

} // namespace nomad
