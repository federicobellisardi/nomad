#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <mutex>
#include <vector>

namespace nomad {

// ── Queue traffic model ───────────────────────────────────────────────────────
// BPR volume-delay function. Occupancy is tracked with a plain counter (events
// are processed sequentially) and exposed as an atomic float via LinkState.
// update() scans only the active-edge dirty set — O(active) not O(E).
class QueueTrafficModel final : public ITrafficModel {
public:
    explicit QueueTrafficModel(const Graph& graph);

    SimTime on_enter    (EdgeId e, AgentId a, SimTime t) override;
    bool    on_exit     (EdgeId e, AgentId a, SimTime t) override;
    void    force_remove(EdgeId e, AgentId a)            override;
    void    update      (SimTime t)                      override;
    float   current_travel_time(EdgeId e) const          override;
    std::span<const LinkState> link_states()  const      override;
    std::string_view model_name()            const       override { return "queue"; }

private:
    struct LinkQueue {
        int32_t  occupancy_count{0};
        float    storage_cap{0.0f};
        float    flow_cap_per_s{0.0f};
        double   last_exit_time{0.0};  // double: sub-second precision at t≈86400s
    };

    const Graph&              graph_;
    std::vector<LinkQueue>    queues_;
    std::vector<LinkState>    states_;

    // Dirty-edge set: tracks edges with occupancy > 0 so update() skips inactive ones.
    std::vector<uint8_t>  is_active_;   // 1 if edge has vehicles
    std::vector<EdgeId>   active_list_; // current active edges (compacted in update)
};

} // namespace nomad
