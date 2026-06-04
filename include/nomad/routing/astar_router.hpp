#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/routing/route_cache.hpp>
#include <nomad/routing/router.hpp>

#include <cstdint>
#include <vector>

namespace nomad {

// Forward declaration — avoids circular include with traffic_model.hpp
class ITrafficModel;

// ── A* router ─────────────────────────────────────────────────────────────────
class AStarRouter final : public IRouter {
public:
    struct Config {
        bool   use_traffic_costs = true;
        float  max_speed_ms      = 33.3f;
    };

    // Two-arg form (convenience overload — default-constructs Config)
    explicit AStarRouter(const Graph& graph,
                          const ITrafficModel* traffic = nullptr);

    // Full constructor
    AStarRouter(const Graph& graph,
                 const ITrafficModel* traffic,
                 Config cfg);

    Route route(const RoutingRequest& req)                               override;
    void  batch_route(std::span<const RoutingRequest>, std::span<Route>) override;
    void  update_costs(std::span<const EdgeId>)                          override {}

    std::string_view router_name() const override { return "A*"; }

    void set_cache(RouteCache* cache) noexcept { cache_ = cache; }

private:
    Route astar_query(NodeId origin, NodeId dest, AgentMode mode) const;

    struct alignas(64) ThreadData {
        std::vector<float>    dist;
        std::vector<NodeId>   prev_node;
        std::vector<EdgeId>   prev_edge;
        std::vector<uint8_t>  visited;
        std::vector<std::pair<float, NodeId>> heap;
        std::vector<NodeId>   touched;  // nodes modified this query — O(touched) reset

        void reset(uint32_t num_nodes);
        void lazy_reset();  // reset only touched nodes, O(touched) not O(N)
    };

    const Graph&             graph_;
    const ITrafficModel*     traffic_;
    Config                   cfg_;
    RouteCache*              cache_{nullptr};
    mutable std::vector<ThreadData> tls_;
};

} // namespace nomad
