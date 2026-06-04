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
    // Allocate one mutex per edge; unique_ptr is movable
    locks_.resize(E);
    for (uint32_t e = 0; e < E; ++e)
        locks_[e] = std::make_unique<std::mutex>();

    for (uint32_t e = 0; e < E; ++e) {
        const EdgeData& ed = graph.edges[e];
        float estimated_lanes = std::max(1.0f, ed.capacity / 1600.0f);
        queues_[e].storage_cap    = ed.length_m * estimated_lanes * kJamDensity;
        queues_[e].flow_cap_per_s = ed.capacity / 3600.0f;
        queues_[e].last_exit_time = 0.0f;
        states_[e].travel_time_s.store(graph.free_flow_time(e));
    }
}

SimTime QueueTrafficModel::on_enter(EdgeId e, AgentId /*a*/, SimTime t) {
    assert(e < graph_.num_edges());
    LinkQueue& lq = queues_[e];
    {
        std::lock_guard lock(*locks_[e]);
        lq.in_transit.push_back(e);
        states_[e].occupancy.store(
            static_cast<float>(lq.in_transit.size() + lq.exit_queue.size()));
    }
    float occ = states_[e].occupancy.load();
    float cap = lq.storage_cap;
    float ff  = graph_.free_flow_time(e);

    if (cap <= 0.0f || occ / cap < 0.8f) {
        states_[e].travel_time_s.store(ff);
        return t + ff;
    }
    float vc = std::min(occ / cap, 1.0f);
    float tt = ff * (1.0f + 0.15f * std::pow(vc, 4.0f));
    states_[e].travel_time_s.store(tt);
    return t + tt;
}

bool QueueTrafficModel::on_exit(EdgeId e, AgentId a, SimTime t) {
    assert(e < graph_.num_edges());
    LinkQueue& lq = queues_[e];
    std::lock_guard lock(*locks_[e]);

    float min_headway = (lq.flow_cap_per_s > 0.0f) ? 1.0f / lq.flow_cap_per_s : 0.0f;
    if (static_cast<float>(t) - lq.last_exit_time < min_headway) {
        lq.exit_queue.push_back(a);
        return false;
    }
    if (!downstream_has_capacity(e)) {
        lq.exit_queue.push_back(a);
        return false;
    }
    if (!lq.in_transit.empty()) lq.in_transit.pop_front();
    lq.last_exit_time = static_cast<float>(t);
    states_[e].occupancy.store(
        static_cast<float>(lq.in_transit.size() + lq.exit_queue.size()));
    states_[e].outflow_rate.store(lq.flow_cap_per_s);
    return true;
}

bool QueueTrafficModel::downstream_has_capacity(EdgeId e) const {
    NodeId target = graph_.edges[e].target;
    for (EdgeId next : graph_.out_edges(target)) {
        float occ = states_[next].occupancy.load();
        float cap = queues_[next].storage_cap;
        if (occ < cap * 0.95f) return true;
    }
    return graph_.out_edges(target).empty();
}

void QueueTrafficModel::update(SimTime /*t*/) {
    for (uint32_t e = 0; e < graph_.num_edges(); ++e) {
        LinkQueue& lq = queues_[e];
        std::lock_guard lock(*locks_[e]);
        float occ = static_cast<float>(lq.in_transit.size() + lq.exit_queue.size());
        states_[e].occupancy.store(occ);
        if (occ == 0.0f)
            states_[e].travel_time_s.store(graph_.free_flow_time(e));
    }
}

float QueueTrafficModel::current_travel_time(EdgeId e) const {
    assert(e < graph_.num_edges());
    return states_[e].travel_time_s.load();
}

std::span<const LinkState> QueueTrafficModel::link_states() const {
    return {states_.data(), states_.size()};
}

} // namespace nomad
