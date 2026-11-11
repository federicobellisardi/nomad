// LTM stuck-time gridlock resolution (LtmTrafficModel::Config::stuck_time_s).
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/traffic/ltm_model.hpp>

using namespace nomad;

// 0 --e0--> 1 --e1 (50 m, 1 lane: storage 6.67)--> 2
static Graph make_chain() {
    Graph g;
    g.nodes.resize(3);
    for (int i = 0; i < 3; ++i) g.nodes[i] = {static_cast<float>(i) * 0.01f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 1, 2, 2};
    g.col_idx = {0, 1};
    auto mk = [](NodeId t) {
        EdgeData d{};
        d.target = t; d.length_m = 50.0f; d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(RoadClass::Primary);
        return d;
    };
    g.edges = {mk(1), mk(2)};
    g.geom_ptr.assign(3, 0);
    return g;
}

static void fill_e1(LtmTrafficModel& m) {
    for (AgentId a = 100; a < 107; ++a) m.on_enter(1, a, 0.0);
    REQUIRE_FALSE(m.has_capacity(1));
}

TEST_CASE("Stuck time: off by default, a spillback block never releases", "[traffic][stuck]") {
    Graph g = make_chain();
    LtmTrafficModel m(g);
    m.on_enter(0, 0, 0.0);
    fill_e1(m);
    for (double t : {0.0, 30.0, 1000.0, 100000.0}) REQUIRE_FALSE(m.on_exit(0, 1, 0, t));
    REQUIRE(m.squeezed_exits() == 0);
}

TEST_CASE("Stuck time: one vehicle per stuck_time_s passes a blocked gate", "[traffic][stuck]") {
    Graph g = make_chain();
    LtmTrafficModel::Config c; c.stuck_time_s = 30.0f;
    LtmTrafficModel m(g, c);
    m.on_enter(0, 0, 0.0);
    m.on_enter(0, 1, 0.0);
    fill_e1(m);
    REQUIRE_FALSE(m.on_exit(0, 1, 0, 5.0));     // block starts at t=5
    REQUIRE_FALSE(m.on_exit(0, 1, 0, 34.0));    // 29 s < 30 s
    REQUIRE(m.on_exit(0, 1, 0, 35.0));          // 30 s: squeezed
    m.on_enter(1, 0, 35.0);                     // e1 now over storage
    REQUIRE(m.squeezed_exits() == 1);
    REQUIRE_FALSE(m.on_exit(0, 1, 1, 35.0));    // timer restarted
    REQUIRE_FALSE(m.on_exit(0, 1, 1, 64.0));
    REQUIRE(m.on_exit(0, 1, 1, 65.0));
    REQUIRE(m.squeezed_exits() == 2);
}

TEST_CASE("Stuck time: unblocked exits are not counted as squeezed", "[traffic][stuck]") {
    Graph g = make_chain();
    LtmTrafficModel::Config c; c.stuck_time_s = 30.0f;
    LtmTrafficModel m(g, c);
    m.on_enter(0, 0, 0.0);
    REQUIRE(m.on_exit(0, 1, 0, 100.0));
    REQUIRE(m.squeezed_exits() == 0);
}
