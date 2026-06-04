#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

using namespace nomad;

// ── Helper: build a small directed graph manually ─────────────────────────────
//
//   0 ──(e0)──► 1 ──(e2)──► 2
//   0 ◄─(e1)── 1 ◄─(e3)── 2
//   0 ──(e4)──► 2 (shortcut)
//   0 ◄─(e5)── 2
//
static Graph make_triangle() {
    Graph g;
    // Nodes: 0=(0,0), 1=(1,0), 2=(2,0)
    g.nodes.resize(3);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};

    // 6 directed edges: 0→1, 1→0, 1→2, 2→1, 0→2, 2→0
    // CSR row_ptr: node 0 has edges 0,4 (e0, e4); node 1 has edges 1,2; node 2 has edges 3,5
    // Let's build it properly with sorted adjacency
    g.row_ptr = {0, 2, 4, 6};  // node 0→[0,2), node 1→[2,4), node 2→[4,6)
    g.col_idx = {0, 4, 1, 2, 3, 5}; // EdgeId by position
    g.edges.resize(6);

    auto make_edge = [](NodeId target, float len, float speed) -> EdgeData {
        EdgeData e{};
        e.target          = target;
        e.length_m        = len;
        e.free_flow_speed = speed;
        e.capacity        = 1600.0f;
        e.road_class      = static_cast<uint8_t>(RoadClass::Primary);
        e.flags           = 0;
        e.way_meta_idx    = 0;
        return e;
    };

    // Row 0: edges e0 (0→1) and e4 (0→2)
    g.edges[0] = make_edge(1, 1000.0f, 10.0f); // e0: 0→1, 100s ff
    g.edges[4] = make_edge(2, 2000.0f, 10.0f); // e4: 0→2, 200s ff

    // Row 1: edges e1 (1→0) and e2 (1→2)
    g.edges[1] = make_edge(0, 1000.0f, 10.0f); // e1: 1→0
    g.edges[2] = make_edge(2, 1000.0f, 10.0f); // e2: 1→2, 100s ff

    // Row 2: edges e3 (2→1) and e5 (2→0)
    g.edges[3] = make_edge(1, 1000.0f, 10.0f); // e3: 2→1
    g.edges[5] = make_edge(0, 2000.0f, 10.0f); // e5: 2→0

    g.geom_ptr.assign(7, 0);
    return g;
}

TEST_CASE("Graph::num_nodes and num_edges", "[graph]") {
    auto g = make_triangle();
    REQUIRE(g.num_nodes() == 3);
    REQUIRE(g.num_edges() == 6);
}

TEST_CASE("Graph::out_edges returns correct span", "[graph]") {
    auto g = make_triangle();
    auto e0 = g.out_edges(0);
    REQUIRE(e0.size() == 2);
    // Node 0 should have edges to node 1 and node 2
    bool found1 = false, found2 = false;
    for (EdgeId eid : e0) {
        NodeId t = g.edges[eid].target;
        if (t == 1) found1 = true;
        if (t == 2) found2 = true;
    }
    REQUIRE(found1);
    REQUIRE(found2);
}

TEST_CASE("Graph::free_flow_time is correct", "[graph]") {
    auto g = make_triangle();
    // Edge 0: 0→1, 1000m / 10 m/s = 100s
    REQUIRE_THAT(g.free_flow_time(0), Catch::Matchers::WithinAbs(100.0f, 0.01f));
}

TEST_CASE("Graph::haversine distance is reasonable", "[graph]") {
    auto g = make_triangle();
    float d = g.haversine(0, 1);
    // Nodes at lon 0 and 1 degree, lat 0: ~111km
    REQUIRE(d > 100'000.0f);
    REQUIRE(d < 120'000.0f);
}

TEST_CASE("Graph::reverse_of is self-inverse", "[graph]") {
    for (EdgeId e = 0; e < 100; ++e)
        REQUIRE(Graph::reverse_of(Graph::reverse_of(e)) == e);
    // Pairs: (0,1), (2,3), (4,5), ...
    REQUIRE(Graph::reverse_of(0) == 1);
    REQUIRE(Graph::reverse_of(1) == 0);
    REQUIRE(Graph::reverse_of(4) == 5);
}

TEST_CASE("Graph CSR invariants: row_ptr is monotone", "[graph]") {
    auto g = make_triangle();
    for (uint32_t i = 0; i + 1 < g.row_ptr.size(); ++i) {
        REQUIRE(g.row_ptr[i] <= g.row_ptr[i + 1]);
    }
}

TEST_CASE("Graph CSR invariants: all edge targets are valid nodes", "[graph]") {
    auto g = make_triangle();
    for (const auto& e : g.edges) {
        REQUIRE(e.target < g.num_nodes());
    }
}

TEST_CASE("Graph::validate passes on well-formed graph", "[graph]") {
    auto g = make_triangle();
    REQUIRE(g.validate());
}

TEST_CASE("EdgeData size is exactly 20 bytes", "[graph]") {
    REQUIRE(sizeof(EdgeData) == 20);
}

TEST_CASE("NodeData size is exactly 16 bytes", "[graph]") {
    REQUIRE(sizeof(NodeData) == 16);
}
