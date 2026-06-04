#include <nomad/traffic/queue_model.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace nomad {

static constexpr float kJamDensity = 1.0f / 7.5f;

QueueTrafficModel::QueueTrafficModel(const Graph& graph) : graph_(graph) {
    const uint32_t E = graph.num_edges();
    queues_.resize(E);
    states_.resize(E);
    locks_.resize(E);
    for (uint32_t e = 0; e < E; ++e)
        locks_[e] = std::make_unique<std::mutex>();

    for (uint32_t e = 0; e < E; ++e) {
        const EdgeData& ed = graph.edges[e];
        float estimated_lanes = std::max(1.0f, ed.capacity / 1600.0f);
        // Minimum storage = 1 vehicle: prevents micro-segments (< 7.5m) from
        // showing false BPR congestion when a single agent enters them.
        queues_[e].storage_cap    = std::max(1.0f, ed.length_m * estimated_lanes * kJamDensity);
        queues_[e].flow_cap_per_s = ed.capacity / 3600.0f;
        queues_[e].last_exit_time = 0.0f;
        states_[e].travel_time_s.store(graph.free_flow_time(e));
    }
}

SimTime QueueTrafficModel::on_enter(EdgeId e, AgentId /*a*/, SimTime t) {
    if (e >= graph_.num_edges()) return t + 1.0;
    LinkQueue& lq = queues_[e];
    {
        std::lock_guard lock(*locks_[e]);
        lq.occupancy_count++;
        states_[e].occupancy.store(static_cast<float>(lq.occupancy_count));
    }
    float occ = states_[e].occupancy.load();
    float cap = lq.storage_cap;
    float ff  = graph_.free_flow_time(e);

    // BPR volume-delay function (Bureau of Public Roads)
    // tt = ff × (1 + 0.15 × (occ/cap)^4)
    // Returns free-flow time when not congested (occ/cap < ~0.8)
    if (cap <= 0.0f || occ / cap < 0.8f) {
        states_[e].travel_time_s.store(ff);
        return t + ff;
    }
    float vc = std::min(occ / cap, 1.0f);
    float tt = ff * (1.0f + 0.15f * std::pow(vc, 4.0f));
    states_[e].travel_time_s.store(tt);
    return t + tt;
}

bool QueueTrafficModel::on_exit(EdgeId e, AgentId /*a*/, SimTime t) {
    // Phase 1: always allow exit; track occupancy and flow rate only.
    // Phase 4 will add proper spillback with per-link FIFO queues and
    // headway enforcement — requires atomic exit_queue flushing.
    if (e >= graph_.num_edges()) return true;
    LinkQueue& lq = queues_[e];
    std::lock_guard lock(*locks_[e]);
    if (lq.occupancy_count > 0) --lq.occupancy_count;
    lq.last_exit_time = static_cast<float>(t);
    states_[e].occupancy.store(static_cast<float>(lq.occupancy_count));
    states_[e].outflow_rate.store(lq.flow_cap_per_s);
    return true;
}

void QueueTrafficModel::update(SimTime /*t*/) {
    for (uint32_t e = 0; e < graph_.num_edges(); ++e) {
        std::lock_guard lock(*locks_[e]);
        float occ = static_cast<float>(queues_[e].occupancy_count);
        states_[e].occupancy.store(occ);
        if (occ == 0.0f)
            states_[e].travel_time_s.store(graph_.free_flow_time(e));
    }
}

float QueueTrafficModel::current_travel_time(EdgeId e) const {
    if (e >= graph_.num_edges()) return 1.0f;
    return states_[e].travel_time_s.load();
}

std::span<const LinkState> QueueTrafficModel::link_states() const {
    return {states_.data(), states_.size()};
}

} // namespace nomad
