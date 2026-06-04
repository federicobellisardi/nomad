#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace nomad {

// ── Routing request ───────────────────────────────────────────────────────────
struct RoutingRequest {
    NodeId    origin;
    NodeId    destination;
    SimTime   departure_time = 0.0;
    AgentMode mode           = AgentMode::Car;
};

// ── Route ─────────────────────────────────────────────────────────────────────
struct Route {
    std::vector<EdgeId> edges;          // ordered sequence of directed edge IDs
    float               estimated_time_s    = 0.0f;
    float               estimated_dist_m    = 0.0f;
    bool                is_cached           = false;
    bool                is_valid            = false;
};

// ── IRouter ───────────────────────────────────────────────────────────────────
// All routing algorithms implement this interface.
// Thread-safety: batch_route is expected to be called concurrently from
// the TBB thread pool; implementations use thread-local data structures
// to avoid contention on shared state.
class IRouter {
public:
    virtual ~IRouter() = default;

    // Single query
    virtual Route route(const RoutingRequest& req) = 0;

    // Batch query (parallel; out must have the same size as reqs)
    virtual void batch_route(std::span<const RoutingRequest> reqs,
                              std::span<Route>               out) = 0;

    // Called by the simulation engine after a traffic state update.
    // Implementations should invalidate affected cache entries and, for CH,
    // update edge costs in the query graph without full re-contraction.
    virtual void update_costs(std::span<const EdgeId> changed_edges) = 0;

    virtual std::string_view router_name() const = 0;
};

} // namespace nomad
