#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/traffic/queue_model.hpp>

using namespace nomad;

// ── Single-edge graph ─────────────────────────────────────────────────────────
static Graph make_single_edge(float length_m = 1000.0f,
                                float speed_ms = 10.0f,
                                float capacity = 1600.0f) {
    Graph g;
    g.nodes.resize(2);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.row_ptr = {0, 1, 2};
    g.col_idx = {0, 1};
    g.edges.resize(2);

    EdgeData fwd{};
    fwd.target = 1; fwd.length_m = length_m;
    fwd.free_flow_speed = speed_ms; fwd.capacity = capacity;
    fwd.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[0] = fwd;

    EdgeData rev = fwd;
    rev.target = 0;
    g.edges[1] = rev;

    g.geom_ptr.assign(3, 0);
    return g;
}

TEST_CASE("QueueModel: on_enter returns free-flow time when empty", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f); // ff_time = 100s
    QueueTrafficModel model(g);
    SimTime exit_time = model.on_enter(0, 0, 0.0);
    REQUIRE_THAT(static_cast<float>(exit_time),
                  Catch::Matchers::WithinAbs(100.0f, 1.0f));
}

TEST_CASE("QueueModel: on_exit succeeds when no spillback", "[traffic]") {
    auto g = make_single_edge(1000.0f, 10.0f);
    QueueTrafficModel model(g);
    model.on_enter(0, 0, 0.0);
    bool ok = model.on_exit(0, 0, 100.0);
    REQUIRE(ok);
}

TEST_CASE("QueueModel: travel time increases with congestion", "[traffic]") {
    auto g = make_single_edge(500.0f, 10.0f, 800.0f); // small cap
    QueueTrafficModel model(g);

    SimTime t0 = model.on_enter(0, 0, 0.0);    // first agent, should be ff
    float ff = 500.0f / 10.0f; // 50s

    // Fill the link close to capacity to see BPR effect
    for (int i = 1; i < 10; ++i) model.on_enter(0, i, 0.0);
    SimTime t1 = model.on_enter(0, 10, 0.0);   // congested entry

    // Congested time should be ≥ free-flow time
    REQUIRE(static_cast<float>(t1) >= static_cast<float>(t0));
}

TEST_CASE("QueueModel: current_travel_time is positive", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.current_travel_time(0) > 0.0f);
    REQUIRE(model.current_travel_time(1) > 0.0f);
}

TEST_CASE("QueueModel: link_states span has correct size", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.link_states().size() == g.num_edges());
}

TEST_CASE("QueueModel: model_name is 'queue'", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    REQUIRE(model.model_name() == "queue");
}

TEST_CASE("QueueModel: occupancy updates after on_enter", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    float before = model.link_states()[0].occupancy.load();
    model.on_enter(0, 0, 0.0);
    float after  = model.link_states()[0].occupancy.load();
    REQUIRE(after > before);
}

TEST_CASE("QueueModel: update does not crash", "[traffic]") {
    auto g = make_single_edge();
    QueueTrafficModel model(g);
    model.on_enter(0, 0, 0.0);
    REQUIRE_NOTHROW(model.update(100.0));
}
