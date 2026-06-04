#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace nomad {

// ── Queue traffic model ───────────────────────────────────────────────────────
// Phase 1: occupancy tracking with BPR volume-delay function.
// on_exit always returns true (no spillback blocking).
//
// Phase 4 roadmap:
//   - Replace occupancy_count with per-link FIFO deque
//   - Add headway enforcement (flow capacity gate)
//   - Add downstream storage capacity check (spillback)
//   - Add proper queue discharge logic
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
        uint32_t occupancy_count{0};  // agents currently on link
        float    storage_cap{0.0f};
        float    flow_cap_per_s{0.0f};
        float    last_exit_time{0.0f};
    };

    const Graph&                                 graph_;
    std::vector<LinkQueue>                       queues_;
    std::vector<std::unique_ptr<std::mutex>>     locks_;
    std::vector<LinkState>                       states_;
};

} // namespace nomad
