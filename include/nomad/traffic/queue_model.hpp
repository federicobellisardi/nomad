#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

namespace nomad {

// ── Queue traffic model (MATSim-style) ────────────────────────────────────────
// Each directed edge is modelled as a FIFO queue with:
//   - free-flow travel time: length_m / free_flow_speed
//   - flow capacity:         EdgeData::capacity [veh/h]
//   - storage capacity:      length_m × lanes × k_jam  (k_jam = 1/7.5 veh/m)
//
// Concurrency: per-link mutexes are stored as unique_ptr<mutex> in a separate
// array (locks_) so that LinkQueue itself remains movable and can be stored in
// std::vector. Both queues_ and locks_ are pre-allocated to the final size in
// the constructor, so no reallocation occurs after construction.
class QueueTrafficModel final : public ITrafficModel {
public:
    explicit QueueTrafficModel(const Graph& graph);

    SimTime on_enter(EdgeId e, AgentId a, SimTime t) override;
    bool    on_exit (EdgeId e, AgentId a, SimTime t) override;
    void    update  (SimTime t)                      override;
    float   current_travel_time(EdgeId e) const      override;
    std::span<const LinkState> link_states()  const  override;
    std::string_view model_name()            const   override { return "queue"; }

private:
    struct LinkQueue {
        std::deque<AgentId> in_transit;
        std::deque<AgentId> exit_queue;
        float storage_cap{0.0f};
        float flow_cap_per_s{0.0f};
        float last_exit_time{0.0f};
        // mutex lives in QueueTrafficModel::locks_ (movable via unique_ptr)
    };

    bool downstream_has_capacity(EdgeId e) const;

    const Graph&                                 graph_;
    std::vector<LinkQueue>                       queues_;
    std::vector<std::unique_ptr<std::mutex>>     locks_;   // one per edge
    std::vector<LinkState>                       states_;
};

} // namespace nomad
