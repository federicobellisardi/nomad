#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_arena.h>
#include <spdlog/spdlog.h>

namespace nomad {

static constexpr float kInf = std::numeric_limits<float>::infinity();

AStarRouter::AStarRouter(const Graph& graph, const ITrafficModel* traffic)
    : AStarRouter(graph, traffic, Config{}) {}

AStarRouter::AStarRouter(const Graph& graph,
                           const ITrafficModel* traffic,
                           Config cfg)
    : graph_(graph), traffic_(traffic), cfg_(cfg)
{
    int nslots = static_cast<int>(std::thread::hardware_concurrency());
    tls_.resize(std::max(nslots, 1));
    for (auto& td : tls_) td.reset(graph.num_nodes());
}

// ── Full reset (constructor only) ─────────────────────────────────────────────
void AStarRouter::ThreadData::reset(uint32_t num_nodes) {
    dist      .assign(num_nodes, kInf);
    prev_node .assign(num_nodes, kInvalidNode);
    prev_edge .assign(num_nodes, kInvalidEdge);
    visited   .assign(num_nodes, 0);
    heap.clear();
    touched.clear();
}

// ── Lazy reset (between queries) ──────────────────────────────────────────────
// Resets only nodes that were touched in the previous query.
// O(touched_nodes) instead of O(N) — 10-40x faster for city-scale graphs
// where A* explores only a fraction of all nodes per query.
void AStarRouter::ThreadData::lazy_reset() {
    for (NodeId n : touched) {
        dist[n]      = kInf;
        visited[n]   = 0;
        prev_node[n] = kInvalidNode;
        prev_edge[n] = kInvalidEdge;
    }
    touched.clear();
    heap.clear();
}

// ── A* query ──────────────────────────────────────────────────────────────────
Route AStarRouter::astar_query(NodeId origin, NodeId dest, AgentMode mode) const {
    const uint32_t N = graph_.num_nodes();
    if (origin >= N || dest >= N) return {};
    if (origin == dest) { Route r; r.is_valid = true; return r; }

    int slot = tbb::this_task_arena::current_thread_index();
    if (slot < 0 || slot >= static_cast<int>(tls_.size())) slot = 0;
    ThreadData& td = const_cast<ThreadData&>(tls_[slot]);

    // Lazy reset — O(previously_touched) instead of O(N)
    td.lazy_reset();

    auto touch = [&](NodeId n) {
        if (td.dist[n] == kInf && !td.visited[n]) td.touched.push_back(n);
    };

    td.dist[origin] = 0.0f;
    td.touched.push_back(origin);

    float h0 = graph_.haversine(origin, dest) / cfg_.max_speed_ms;
    td.heap.push_back({h0, origin});

    auto cmp = [](const std::pair<float,NodeId>& a, const std::pair<float,NodeId>& b){
        return a.first > b.first;
    };
    std::make_heap(td.heap.begin(), td.heap.end(), cmp);

    while (!td.heap.empty()) {
        std::pop_heap(td.heap.begin(), td.heap.end(), cmp);
        auto [f, u] = td.heap.back();
        td.heap.pop_back();

        if (td.visited[u]) continue;
        td.visited[u] = 1;

        if (u == dest) break;

        float g_u = td.dist[u];

        for (EdgeId eid : graph_.out_edges(u)) {
            const EdgeData& ed = graph_.edges[eid];
            NodeId v = ed.target;
            if (v >= N) continue;
            if (td.visited[v]) continue;
            // Mode filter: skip edges not accessible to this agent's mode
            if (!road_class_accessible(static_cast<RoadClass>(ed.road_class), mode))
                continue;

            float cost = (traffic_ && cfg_.use_traffic_costs)
                ? traffic_->current_travel_time(eid)
                : graph_.free_flow_time(eid);
            if (cost <= 0.0f) cost = graph_.free_flow_time(eid);

            float g_v = g_u + cost;
            if (g_v < td.dist[v]) {
                touch(v);
                td.dist[v]      = g_v;
                td.prev_node[v] = u;
                td.prev_edge[v] = eid;
                float h_v = graph_.haversine(v, dest) / cfg_.max_speed_ms;
                td.heap.push_back({g_v + h_v, v});
                std::push_heap(td.heap.begin(), td.heap.end(), cmp);
            }
        }
    }

    if (td.dist[dest] == kInf) return {};

    Route route;
    route.is_valid = true;
    route.estimated_time_s = td.dist[dest];

    NodeId cur = dest;
    while (cur != origin) {
        EdgeId eid = td.prev_edge[cur];
        if (eid == kInvalidEdge) return {}; // disconnected path — shouldn't happen
        route.edges.push_back(eid);
        route.estimated_dist_m += graph_.edges[eid].length_m;
        cur = td.prev_node[cur];
        if (cur == kInvalidNode) return {};
    }
    std::reverse(route.edges.begin(), route.edges.end());
    return route;
}

Route AStarRouter::route(const RoutingRequest& req) {
    if (cache_) {
        if (auto cached = cache_->get(req.origin, req.destination,
                                       req.mode, req.departure_time)) {
            return *cached;
        }
    }
    Route r = astar_query(req.origin, req.destination, req.mode);
    if (r.is_valid && cache_) {
        cache_->put(req.origin, req.destination, req.mode, req.departure_time, r);
    }
    return r;
}

void AStarRouter::batch_route(std::span<const RoutingRequest> reqs,
                                std::span<Route> out) {
    assert(reqs.size() == out.size());
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, reqs.size()),
        [&](const tbb::blocked_range<std::size_t>& r) {
            for (std::size_t i = r.begin(); i < r.end(); ++i)
                out[i] = route(reqs[i]);
        });
}

} // namespace nomad
