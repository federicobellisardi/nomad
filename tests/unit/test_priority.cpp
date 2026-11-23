// LTM node priority (LtmTrafficModel::Config::priority_reserve_veh).
// Each link has one major approach (roundabout circulating link if the link
// is on a roundabout, otherwise the best road class, ties broken by the
// straightest continuation). Vehicles from any other approach, and departures,
// enter only while the receiving link keeps more than priority_reserve_veh
// free storage. Without it side entries took every freed slot and a ring of
// links (dual carriageway + U-turns) reached jam density and deadlocked.
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/traffic/ltm_model.hpp>

using namespace nomad;

namespace {
//   main 0 --e0 (primary)----> 1 --e2 (primary, 50 m)--> 3 --e3--> 4
//   side 2 --e1 (residential)-^
// node coords: 0 (0,0), 1 (0.01,0), 3 (0.02,0), 4 (0.03,0), 2 (0.01,-0.01)
Graph make_merge(bool roundabout_side = false) {
    Graph g;
    g.nodes.resize(5);
    const float xy[5][2] = {{0.0f, 0.0f}, {0.01f, 0.0f}, {0.01f, -0.01f}, {0.02f, 0.0f}, {0.03f, 0.0f}};
    for (int i = 0; i < 5; ++i) g.nodes[i] = {xy[i][0], xy[i][1], 0, 0, 0, static_cast<uint32_t>(i)};
    // out-edges: node0: e0; node1: e2; node2: e1; node3: e3
    g.row_ptr = {0, 1, 2, 3, 4, 4};
    g.col_idx = {0, 2, 1, 3};
    auto mk = [](NodeId t, float len, RoadClass rc, uint8_t fl = 0) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(rc); d.flags = fl;
        return d;
    };
    g.edges = {mk(1, 200.0f, RoadClass::Primary),
               mk(1, 200.0f, RoadClass::Residential, roundabout_side ? EdgeFlags::Roundabout : 0),
               mk(3, 50.0f, RoadClass::Primary, roundabout_side ? EdgeFlags::Roundabout : 0),
               mk(4, 200.0f, RoadClass::Primary)};
    g.geom_ptr.assign(5, 0);
    return g;
}

LtmTrafficModel make(const Graph& g, float reserve, float stuck = 0.0f) {
    LtmTrafficModel::Config c;
    c.priority_reserve_veh = reserve;
    c.stuck_time_s = stuck;
    return LtmTrafficModel(g, c);
}

// e2 storage = 50 m x 1 lane / 7.5 = 6.67 veh. With 5 on it, free = 1.67.
void fill_receiver(LtmTrafficModel& m, int n) {
    for (int i = 0; i < n; ++i) m.on_enter(2, 100 + i, 0.0);
}
} // namespace

TEST_CASE("Priority: major approach is the better road class", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 2.0f);
    REQUIRE(m.is_major(0, 2));
    REQUIRE_FALSE(m.is_major(1, 2));
}

TEST_CASE("Priority: minor entry yields when reserve is reached, major does not", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 2.0f);
    m.on_enter(0, 1, 0.0);
    m.on_enter(1, 2, 0.0);
    fill_receiver(m, 5);                          // free 1.67 <= 2
    REQUIRE_FALSE(m.on_exit_to(1, 2, 3, 2, 10.0)); // side waits
    REQUIRE(m.on_exit_to(0, 2, 3, 1, 10.0));       // main enters
}

TEST_CASE("Priority: minor entry admitted while reserve is free", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 2.0f);
    m.on_enter(1, 2, 0.0);
    fill_receiver(m, 4);                          // free 2.67 > 2
    REQUIRE(m.on_exit_to(1, 2, 3, 2, 10.0));
}

TEST_CASE("Priority: off by default", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 0.0f);
    m.on_enter(1, 2, 0.0);
    fill_receiver(m, 5);
    REQUIRE(m.on_exit_to(1, 2, 3, 2, 10.0));
    REQUIRE(m.has_capacity_departure(2, 3));
}

TEST_CASE("Priority: departures yield like minor entries", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 2.0f);
    fill_receiver(m, 4);
    REQUIRE(m.has_capacity_departure(2, 3));
    m.on_enter(2, 200, 0.0);                      // 5 on it
    REQUIRE_FALSE(m.has_capacity_departure(2, 3));
    REQUIRE(m.has_capacity_for(2, 3));            // plain storage check unchanged
}

TEST_CASE("Priority: roundabout circulating link is major over a better class", "[traffic][priority]") {
    Graph g = make_merge(true);
    auto m = make(g, 2.0f);
    REQUIRE(m.is_major(1, 2));
    REQUIRE_FALSE(m.is_major(0, 2));
}

TEST_CASE("Priority: stuck-time squeeze still overrides a minor block", "[traffic][priority]") {
    Graph g = make_merge();
    auto m = make(g, 2.0f, 30.0f);
    m.on_enter(1, 2, 0.0);
    fill_receiver(m, 5);
    REQUIRE_FALSE(m.on_exit_to(1, 2, 3, 2, 10.0)); // block starts at t=10
    REQUIRE_FALSE(m.on_exit_to(1, 2, 3, 2, 30.0));
    REQUIRE(m.on_exit_to(1, 2, 3, 2, 41.0));       // >= 30 s blocked
}

// ── Link yield rule and priority-ordered service (priority_service) ─────────
namespace {
Graph make_ramp_merge() {           // side approach is a motorway_link joining a primary
    Graph g = make_merge();
    g.edges[1].road_class = static_cast<uint8_t>(RoadClass::MotorwayLink);
    return g;
}
LtmTrafficModel make_service(const Graph& g) {
    LtmTrafficModel::Config c;
    c.priority_service = true;
    return LtmTrafficModel(g, c);
}
} // namespace

TEST_CASE("Priority: a *_link approach yields to the through road despite its class", "[traffic][priority]") {
    Graph g = make_ramp_merge();
    auto m = make(g, 2.0f);
    REQUIRE(m.is_major(0, 2));          // primary through road
    REQUIRE_FALSE(m.is_major(1, 2));    // motorway_link ramp
}

TEST_CASE("Priority service: major approaches are flagged per node, no storage reserve", "[traffic][priority][service]") {
    Graph g = make_ramp_merge();
    auto m = make_service(g);
    REQUIRE(m.priority_service());
    REQUIRE(m.is_major_at_node(0));
    REQUIRE_FALSE(m.is_major_at_node(1));
    // no reserve: a minor entry is admitted whenever the receiver has room
    m.on_enter(1, 2, 0.0);
    fill_receiver(m, 5);                // free 1.67
    REQUIRE(m.on_exit_to(1, 2, 3, 2, 10.0));
    REQUIRE(m.has_capacity_departure(2, 3));
}

TEST_CASE("Priority service: off by default", "[traffic][priority][service]") {
    Graph g = make_ramp_merge();
    LtmTrafficModel m(g);
    REQUIRE_FALSE(m.priority_service());
    REQUIRE(m.is_major_at_node(1));     // every approach counts as major when off
}

// ── Unmapped roundabouts: short one-way cycles (priority_cycle_max_m) ───────
namespace {
// Square one-way ring 0->1->2->3->0 (sides 50 m, primary), plus a straight
// two-way-free approach 4->1 (primary, 200 m) entering the ring at node 1
// in line with ring link 1->2 (the entry is the straightest continuation).
//   coordinates: 0 (0,0) 1 (0.0006,0) 2 (0.0006,0.00045) 3 (0,0.00045) 4 (0.0006,-0.0018)
Graph make_ring(float side = 50.0f) {
    Graph g;
    g.nodes.resize(5);
    const float xy[5][2] = {{0.0f, 0.0f}, {0.0006f, 0.0f}, {0.0006f, 0.00045f}, {0.0f, 0.00045f}, {0.0006f, -0.0018f}};
    for (int i = 0; i < 5; ++i) g.nodes[i] = {xy[i][0], xy[i][1], 0, 0, 0, static_cast<uint32_t>(i)};
    // out-edges: node0: e0 (0->1); node1: e1 (1->2); node2: e2 (2->3); node3: e3 (3->0); node4: e4 (4->1)
    g.row_ptr = {0, 1, 2, 3, 4, 5};
    g.col_idx = {0, 1, 2, 3, 4};
    auto mk = [](NodeId t, float len) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(RoadClass::Primary);
        return d;
    };
    g.edges = {mk(1, side), mk(2, side), mk(3, side), mk(0, side), mk(1, 200.0f)};
    g.geom_ptr.assign(6, 0);
    return g;
}
LtmTrafficModel make_cycle(const Graph& g, float max_m) {
    LtmTrafficModel::Config c;
    c.priority_service = true;
    c.priority_cycle_max_m = max_m;
    return LtmTrafficModel(g, c);
}
} // namespace

TEST_CASE("Priority: without cycle detection the straight entry wins on a one-way ring", "[traffic][priority][cycle]") {
    Graph g = make_ring();
    auto m = make_cycle(g, 0.0f);
    REQUIRE(m.is_major(4, 1));
    REQUIRE_FALSE(m.is_major(0, 1));
    REQUIRE(m.circulating_links() == 0);
}

TEST_CASE("Priority: a short one-way cycle is treated as a roundabout (circulating link major)", "[traffic][priority][cycle]") {
    Graph g = make_ring();
    auto m = make_cycle(g, 500.0f);
    REQUIRE(m.circulating_links() == 4);
    REQUIRE(m.is_major(0, 1));          // circulating
    REQUIRE_FALSE(m.is_major(4, 1));    // entry yields
}

TEST_CASE("Priority: cycles longer than the threshold are not roundabouts", "[traffic][priority][cycle]") {
    Graph g = make_ring(200.0f);        // perimeter 800 m
    auto m = make_cycle(g, 500.0f);
    REQUIRE(m.circulating_links() == 0);
    REQUIRE(m.is_major(4, 1));
}

TEST_CASE("Priority: a cycle through a signalised node is not a roundabout", "[traffic][priority][cycle]") {
    Graph g = make_ring();
    g.nodes[2].intersection_type = static_cast<uint8_t>(IntersectionType::TrafficSignal);
    auto m = make_cycle(g, 500.0f);
    REQUIRE(m.circulating_links() == 0);
}

TEST_CASE("Priority: two-way road pairs are not cycles", "[traffic][priority][cycle]") {
    Graph g = make_merge();             // add the reverse of e2 (3->1): e2 becomes two-way
    g.row_ptr = {0, 1, 2, 3, 5, 5};
    g.col_idx = {0, 2, 1, 3, 4};
    EdgeData r = g.edges[2]; r.target = 1; g.edges.push_back(r);
    g.geom_ptr.assign(6, 0);
    auto m = make_cycle(g, 500.0f);
    REQUIRE(m.circulating_links() == 0);
}

TEST_CASE("Priority: one-way residential blocks are not roundabouts", "[traffic][priority][cycle]") {
    Graph g = make_ring();
    for (int i = 0; i < 4; ++i) g.edges[i].road_class = static_cast<uint8_t>(RoadClass::Residential);
    auto m = make_cycle(g, 500.0f);
    REQUIRE(m.circulating_links() == 0);
}

// ── Squeeze of minor approaches only into stuck receivers ───────────────────
namespace {
LtmTrafficModel make_minor_squeeze(const Graph& g, bool stuck_only) {
    LtmTrafficModel::Config c;
    c.priority_service = true;
    c.stuck_time_s = 30.0f;
    c.minor_squeeze_stuck_receiver = stuck_only;
    return LtmTrafficModel(g, c);
}
// Receiver e2 full (7 >= 6.67); approach `from` blocked from t=10; e2 drains
// one vehicle at t=35 and is refilled. Returns admissions at t=41 and t=66.
std::pair<bool, bool> squeeze_case(bool stuck_only, EdgeId from) {
    Graph g = make_ramp_merge();
    auto m = make_minor_squeeze(g, stuck_only);
    m.on_enter(from, 1, 0.0);
    fill_receiver(m, 7);
    REQUIRE_FALSE(m.on_exit_to(from, 2, 3, 1, 10.0));       // block starts
    REQUIRE(m.on_exit_to(2, 3, kInvalidEdge, 100, 35.0));   // receiver still drains
    m.on_enter(2, 300, 35.0);                                // and is full again
    const bool at41 = m.on_exit_to(from, 2, 3, 1, 41.0);
    const bool at66 = at41 ? true : m.on_exit_to(from, 2, 3, 1, 66.0);
    return {at41, at66};
}
} // namespace

TEST_CASE("Minor squeeze: without the rule a waiting minor entry is squeezed after stuck_time", "[traffic][priority][squeeze]") {
    auto [at41, at66] = squeeze_case(false, 1);
    REQUIRE(at41);
}

TEST_CASE("Minor squeeze: a minor entry is not squeezed into a receiver that still drains", "[traffic][priority][squeeze]") {
    auto [at41, at66] = squeeze_case(true, 1);
    REQUIRE_FALSE(at41);   // receiver exited at 35: give-way wait, not gridlock
    REQUIRE(at66);         // no exit for 31 s: true deadlock, squeeze allowed
}

TEST_CASE("Minor squeeze: major approaches keep the plain stuck-time squeeze", "[traffic][priority][squeeze]") {
    auto [at41, at66] = squeeze_case(true, 0);
    REQUIRE(at41);
}
