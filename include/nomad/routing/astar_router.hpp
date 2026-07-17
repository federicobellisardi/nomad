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
        // Mode-specific speed caps, applied via Graph::mode_free_flow_time.
        // Defaults mirror ModeChoiceConfig (scenario_config.hpp).
        float  walk_speed_ms     = 1.39f;   // 5 km/h
        float  bike_speed_ms     = 4.17f;   // 15 km/h
    };

    // Two-arg form (convenience overload — default-constructs Config)
    explicit AStarRouter(const Graph& graph,
                          const ITrafficModel* traffic = nullptr);

    // Full constructor
    AStarRouter(const Graph& graph,
                 const ITrafficModel* traffic,
                 Config cfg);

    // Per-class cost multipliers: index = RoadClass uint8 value (0–18).
    // Unknown (255) falls back to 1.0. Used for stochastic pre-routing.
    static constexpr size_t kNumRoadClasses = 19;
    using ClassMultipliers = std::array<float, kNumRoadClasses>;

    Route route(const RoutingRequest& req)                               override;
    void  batch_route(std::span<const RoutingRequest>, std::span<Route>) override;
    void  update_costs(std::span<const EdgeId>)                          override {}

    // Route with per-road-class cost multipliers (stochastic pre-routing).
    // cost(e) = free_flow_time(e) * class_mult[road_class(e)]
    Route route_perturbed(NodeId origin, NodeId dest, AgentMode mode,
                           const ClassMultipliers& class_mult) const;

    // Per-edge stochastic routing: each edge gets a deterministic per-agent
    // noise multiplier derived from hash(agent_seed, edge_id).
    // cost(e) = free_flow_time(e) * (1 + sigma * u)  where u ∈ [-1, 1]
    // sigma = 0.05–0.10 produces route diversity across nearby junction
    // alternatives without major detours. Heuristic scaled by (1-sigma) to
    // remain admissible. Taylor approx avoids exp() for sigma <= 0.15.
    Route route_stochastic(NodeId origin, NodeId dest, AgentMode mode,
                            uint32_t agent_seed, float sigma) const;

    std::string_view router_name() const override { return "A*"; }

    void set_cache(RouteCache* cache) noexcept { cache_ = cache; }

private:
    Route astar_query(NodeId origin, NodeId dest, AgentMode mode) const;

    // Traffic-aware-or-free-flow cost for Car (existing behaviour); mode-capped
    // free-flow cost for Walk/Bike (never traffic-aware — no ped/bike congestion
    // model). Shared by astar_query / route_perturbed / route_stochastic.
    float edge_cost(EdgeId e, AgentMode mode) const;

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
