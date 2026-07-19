#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/network/network_cleaner.hpp>

using namespace nomad;

static EdgeData make_edge(NodeId target, float len, float speed = 10.0f,
                            float capacity = 1600.0f) {
    EdgeData e{};
    e.target          = target;
    e.length_m        = len;
    e.free_flow_speed = speed;
    e.capacity        = capacity;
    e.road_class      = static_cast<uint8_t>(RoadClass::Motorway);
    e.flags           = 0;
    e.way_meta_idx    = 0;
    return e;
}

// A permissive config so remove_small_components doesn't discard these
// tiny test graphs (default min_component_size=5).
static NetworkCleaner::Config permissive_cfg() {
    NetworkCleaner::Config cfg;
    cfg.min_component_size = 1;
    return cfg;
}

// One-way chain 0 → 1 → 2 (no reverse edges at all — the common real-world
// shape for a motorway/trunk carriageway in OSM). Node 1 has exactly one
// in-edge (from 0) and one out-edge (to 2): a true degree-2 pass-through
// that should be contracted into a single 0→2 edge.
TEST_CASE("NetworkCleaner: contracts a one-way degree-2 chain", "[network]") {
    Graph g;
    g.nodes.resize(3);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};
    g.row_ptr = {0, 1, 2, 2};
    g.col_idx = {0, 1};
    g.edges.resize(2);
    g.edges[0] = make_edge(1, 100.0f); // 0→1
    g.edges[1] = make_edge(2, 150.0f); // 1→2
    g.geom_ptr.assign(3, 0);

    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(std::move(g));

    REQUIRE(out.num_nodes() == 2);
    REQUIRE(out.num_edges() == 1);
    REQUIRE_THAT(out.edges[0].length_m, Catch::Matchers::WithinAbs(250.0f, 0.01f));
}

// Bidirectional chain 0 ↔ 1 ↔ 2 (forward + reverse pair at each segment).
// Regression guard: this case already worked before the one-way fix.
TEST_CASE("NetworkCleaner: contracts a bidirectional degree-2 chain", "[network]") {
    Graph g;
    g.nodes.resize(3);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};
    g.row_ptr = {0, 1, 3, 4};
    g.col_idx = {0, 1, 2, 3};
    g.edges.resize(4);
    g.edges[0] = make_edge(1, 100.0f); // 0→1
    g.edges[1] = make_edge(0, 100.0f); // 1→0
    g.edges[2] = make_edge(2, 150.0f); // 1→2
    g.edges[3] = make_edge(1, 150.0f); // 2→1
    g.geom_ptr.assign(5, 0);

    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(std::move(g));

    REQUIRE(out.num_nodes() == 2);
    REQUIRE(out.num_edges() == 2); // one merged edge per direction
}

// Star junction: node 1 has three distinct neighbours (0, 2, 3) — a genuine
// intersection, not a pass-through. Must NOT be contracted.
TEST_CASE("NetworkCleaner: does not contract a real 3-way junction", "[network]") {
    Graph g;
    g.nodes.resize(4);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};
    g.nodes[3] = {1.0f, 1.0f, 0, 0, 0, 3};
    g.row_ptr = {0, 1, 3, 3, 3};
    g.col_idx = {0, 1, 2};
    g.edges.resize(3);
    g.edges[0] = make_edge(1, 100.0f); // 0→1
    g.edges[1] = make_edge(2, 100.0f); // 1→2
    g.edges[2] = make_edge(3, 100.0f); // 1→3
    g.geom_ptr.assign(4, 0);

    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(std::move(g));

    REQUIRE(out.num_nodes() == 4);
    REQUIRE(out.num_edges() == 3);
}

// Regression test for a real bug found running this on Barcelona's ~2M-node
// graph: simplify_degree2_nodes could re-contract the same node an unbounded
// number of times (worklist_size stabilized at 1 while the contracted
// counter grew without bound -- confirmed via instrumentation, and by
// running the actual Barcelona extract, which never terminated before the
// fix and completes in ~1s after it). A node can structurally only be
// contracted once (it has no valid edges left afterwards), so
// nodes_simplified must never exceed nodes_before -- this is exactly the
// invariant that was violated. Exercised here on a closed ring where every
// edge is shorter than min_edge_length_m (forcing repeated Criterion-B
// eligibility checks throughout the contraction, the code path involved),
// large enough that the old unbounded-growth bug would not self-terminate
// within any reasonable test timeout.
TEST_CASE("NetworkCleaner: nodes_simplified never exceeds nodes_before (closed ring, short edges)", "[network]") {
    const uint32_t N = 50;
    Graph g;
    g.nodes.resize(N);
    g.row_ptr.resize(N + 1);
    g.edges.resize(2 * N);
    g.col_idx.resize(2 * N);
    for (uint32_t i = 0; i < N; ++i) {
        g.nodes[i] = {static_cast<float>(i), 0.0f, 0, 0, 0, i};
        uint32_t next = (i + 1) % N;
        uint32_t prev = (i + N - 1) % N;
        g.row_ptr[i] = 2 * i;
        g.edges[2 * i]     = make_edge(next, 5.0f);  // shorter than default min_edge_length_m (20m)
        g.edges[2 * i + 1] = make_edge(prev, 5.0f);
        g.col_idx[2 * i]     = 2 * i;
        g.col_idx[2 * i + 1] = 2 * i + 1;
    }
    g.row_ptr[N] = 2 * N;
    g.geom_ptr.assign(2 * N + 1, 0);

    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(std::move(g));

    const auto& report = cleaner.last_report();
    REQUIRE(report.nodes_simplified <= report.nodes_before);
    REQUIRE(out.num_nodes() >= 1);
}
