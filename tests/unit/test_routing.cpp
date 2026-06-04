#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/routing/route_cache.hpp>

using namespace nomad;

// ── Toy graph: linear chain 0→1→2→3→4 (bidirectional) ───────────────────────
static Graph make_chain(int n = 5) {
    Graph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) {
        g.nodes[i].lon = static_cast<float>(i);
        g.nodes[i].lat = 0.0f;
    }

    // Build CSR: each node i has edges to i-1 (if exists) and i+1 (if exists)
    g.row_ptr.resize(n + 1, 0);
    for (int i = 0; i < n; ++i) {
        int deg = 0;
        if (i > 0)   ++deg;
        if (i < n-1) ++deg;
        g.row_ptr[i + 1] = g.row_ptr[i] + deg;
    }

    uint32_t E = g.row_ptr[n];
    g.edges.resize(E);
    g.col_idx.resize(E);

    uint32_t eid = 0;
    for (int i = 0; i < n; ++i) {
        if (i > 0) {
            EdgeData ed{};
            ed.target          = static_cast<NodeId>(i - 1);
            ed.length_m        = 1000.0f;
            ed.free_flow_speed = 10.0f;   // 100s per edge
            ed.capacity        = 1600.0f;
            g.edges[eid] = ed;
            g.col_idx[eid] = eid;
            ++eid;
        }
        if (i < n - 1) {
            EdgeData ed{};
            ed.target          = static_cast<NodeId>(i + 1);
            ed.length_m        = 1000.0f;
            ed.free_flow_speed = 10.0f;
            ed.capacity        = 1600.0f;
            g.edges[eid] = ed;
            g.col_idx[eid] = eid;
            ++eid;
        }
    }
    g.geom_ptr.assign(E + 1, 0);
    return g;
}

TEST_CASE("A* finds path on chain graph", "[routing]") {
    auto g = make_chain(5);
    AStarRouter router(g);
    RoutingRequest req{0, 4, 0.0, AgentMode::Car};
    auto route = router.route(req);
    REQUIRE(route.is_valid);
    REQUIRE(route.edges.size() == 4);  // 4 edges to traverse 5 nodes
}

TEST_CASE("A* origin == destination returns empty valid route", "[routing]") {
    auto g = make_chain(5);
    AStarRouter router(g);
    RoutingRequest req{2, 2, 0.0, AgentMode::Car};
    auto route = router.route(req);
    REQUIRE(route.is_valid);
    REQUIRE(route.edges.empty());
}

TEST_CASE("A* returns invalid route for disconnected graph", "[routing]") {
    // Isolated node 5 added but not connected
    auto g = make_chain(5);
    g.nodes.push_back({10.0f, 10.0f, 0, 0, 0, 5});
    g.row_ptr.push_back(g.row_ptr.back()); // degree 0
    AStarRouter router(g);
    RoutingRequest req{0, 5, 0.0, AgentMode::Car};
    auto route = router.route(req);
    REQUIRE_FALSE(route.is_valid);
}

TEST_CASE("A* estimated_time_s matches manual calculation", "[routing]") {
    auto g = make_chain(3);  // 0→1→2, each edge 100s
    AStarRouter router(g);
    RoutingRequest req{0, 2, 0.0, AgentMode::Car};
    auto route = router.route(req);
    REQUIRE(route.is_valid);
    // 2 edges × 1000m / 10 m/s = 200s
    REQUIRE_THAT(route.estimated_time_s, Catch::Matchers::WithinAbs(200.0f, 1.0f));
}

TEST_CASE("A* batch route is consistent with single route", "[routing]") {
    auto g = make_chain(5);
    AStarRouter router(g);
    RoutingRequest reqs[3] = {
        {0, 4, 0.0, AgentMode::Car},
        {1, 3, 0.0, AgentMode::Car},
        {0, 2, 0.0, AgentMode::Car},
    };
    Route out[3];
    router.batch_route(std::span(reqs, 3), std::span(out, 3));
    REQUIRE(out[0].is_valid);
    REQUIRE(out[1].is_valid);
    REQUIRE(out[2].is_valid);
    // Single reference
    auto ref = router.route({0, 4, 0.0, AgentMode::Car});
    REQUIRE(out[0].edges.size() == ref.edges.size());
}

TEST_CASE("RouteCache: hit rate after insertion", "[routing][cache]") {
    RouteCache cache(1000);
    Route r;
    r.edges = {0, 2, 4};
    r.estimated_time_s = 300.0f;
    r.is_valid = true;

    cache.put(0, 10, AgentMode::Car, 28800.0, r);
    auto hit = cache.get(0, 10, AgentMode::Car, 28800.0);
    REQUIRE(hit.has_value());
    REQUIRE(hit->edges.size() == 3);

    auto miss = cache.get(0, 10, AgentMode::Bike, 28800.0);
    REQUIRE_FALSE(miss.has_value());

    auto stats = cache.stats();
    REQUIRE(stats.hits == 1);
    REQUIRE(stats.misses == 1);
}

TEST_CASE("RouteCache: TOD buckets group nearby times", "[routing][cache]") {
    RouteCache cache(100);
    Route r;
    r.edges = {0}; r.is_valid = true;
    // Same 15-min bucket
    cache.put(0, 5, AgentMode::Car, 28800.0, r);
    REQUIRE(cache.get(0, 5, AgentMode::Car, 28801.0).has_value()); // same bucket
    REQUIRE_FALSE(cache.get(0, 5, AgentMode::Car, 29700.0).has_value()); // different bucket
}

TEST_CASE("RouteCache: invalidate removes affected entries", "[routing][cache]") {
    RouteCache cache(100);
    Route r;
    r.edges = {0, 2, 4}; r.is_valid = true;
    cache.put(0, 5, AgentMode::Car, 0.0, r);

    EdgeId changed[] = {2};
    cache.invalidate_edges(std::span(changed, 1));
    REQUIRE_FALSE(cache.get(0, 5, AgentMode::Car, 0.0).has_value());
}

TEST_CASE("RouteCache: LRU eviction", "[routing][cache]") {
    RouteCache cache(3);  // max 3 entries
    Route r; r.is_valid = true;
    cache.put(0, 1, AgentMode::Car, 0.0, r);
    cache.put(0, 2, AgentMode::Car, 0.0, r);
    cache.put(0, 3, AgentMode::Car, 0.0, r);
    cache.put(0, 4, AgentMode::Car, 0.0, r);  // evicts LRU (dest=1)
    REQUIRE(cache.size() == 3);
}
