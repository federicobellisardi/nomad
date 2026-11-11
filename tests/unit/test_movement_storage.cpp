// LTM movement-share storage (LtmTrafficModel::Config::movement_storage).
// Vehicles on link e heading to next link n may occupy at most
// storage(e) * min(1, lanes(n)/lanes(e)) of e's storage, so a queue for a
// narrow movement (e.g. a 1-lane off-ramp) cannot fill a wide link and block
// the other movements (diverge blocking).
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <filesystem>
#include <fstream>
#include <unordered_map>

using namespace nomad;

// Diverge: 0 --e0 (4 lanes)--> 1 --e1 (4 lanes)--> 2      (through)
//                                1 --e2 (1 lane) --> 3 --e3 (tiny)--> 4 (exit)
// CSR positions deliberately differ from EdgeIds at node 1 (col_idx non-identity).
static Graph make_diverge(float e0_len = 50.0f) {
    Graph g;
    g.nodes.resize(5);
    for (int i = 0; i < 5; ++i) g.nodes[i] = {static_cast<float>(i) * 0.01f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 1, 3, 4, 4, 4};
    g.col_idx = {0, 2, 1, 3};
    auto mk = [](NodeId t, float len, float cap) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 20.0f; d.capacity = cap;
        d.road_class = static_cast<uint8_t>(RoadClass::Primary);
        return d;
    };
    g.edges = {mk(1, e0_len, 6400.0f),   // e0 approach, 4 lanes
               mk(2, 200.0f, 6400.0f),   // e1 through, 4 lanes
               mk(3, 100.0f, 1600.0f),   // e2 exit ramp, 1 lane
               mk(4, 100.0f, 60.0f)};    // e3 bottleneck after the ramp
    g.geom_ptr.assign(5, 0);
    return g;
}

static LtmTrafficModel make_ltm(const Graph& g, bool movement_storage) {
    LtmTrafficModel::Config c;
    c.movement_storage = movement_storage;
    return LtmTrafficModel(g, c);
}

// e0: storage = 50 m * 4 lanes / 7.5 = 26.67; exit share = 1/4 -> 6.67.
TEST_CASE("Movement storage: a narrow movement saturates only its share", "[traffic][movement]") {
    Graph g = make_diverge();
    auto m = make_ltm(g, true);
    for (AgentId a = 0; a < 7; ++a) {
        REQUIRE(m.has_capacity_for(0, 2));
        m.on_enter(0, a, 0.0);
        m.assign_movement(0, a, 2);
    }
    REQUIRE_FALSE(m.has_capacity_for(0, 2));  // exit share full
    REQUIRE(m.has_capacity_for(0, 1));        // through still admitted
    REQUIRE(m.has_capacity(0));
}

TEST_CASE("Movement storage: disabled by default, only total storage binds", "[traffic][movement]") {
    Graph g = make_diverge();
    auto m = make_ltm(g, false);
    for (AgentId a = 0; a < 20; ++a) {
        m.on_enter(0, a, 0.0);
        m.assign_movement(0, a, 2);
    }
    REQUIRE(m.has_capacity_for(0, 2));
}

TEST_CASE("Movement storage: total storage still binds", "[traffic][movement]") {
    Graph g = make_diverge();
    auto m = make_ltm(g, true);
    for (AgentId a = 0; a < 27; ++a) {
        m.on_enter(0, a, 0.0);
        m.assign_movement(0, a, 1);
    }
    REQUIRE_FALSE(m.has_capacity_for(0, 1));
    REQUIRE_FALSE(m.has_capacity_for(0, 2));
}

TEST_CASE("Movement storage: exit, teleport and reassignment release the slot", "[traffic][movement]") {
    Graph g = make_diverge();
    auto m = make_ltm(g, true);
    for (AgentId a = 0; a < 7; ++a) { m.on_enter(0, a, 0.0); m.assign_movement(0, a, 2); }
    REQUIRE_FALSE(m.has_capacity_for(0, 2));

    REQUIRE(m.on_exit(0, 2, 0, 10.0));          // agent 0 leaves onto the ramp
    REQUIRE(m.has_capacity_for(0, 2));
    m.on_enter(0, 7, 10.0); m.assign_movement(0, 7, 2);
    REQUIRE_FALSE(m.has_capacity_for(0, 2));

    m.force_remove(0, 1);                       // teleported
    REQUIRE(m.has_capacity_for(0, 2));
    m.on_enter(0, 8, 10.0); m.assign_movement(0, 8, 2);
    REQUIRE_FALSE(m.has_capacity_for(0, 2));

    m.assign_movement(0, 2, 1);                 // rerouted on the link: now through
    REQUIRE(m.has_capacity_for(0, 2));
}

TEST_CASE("Movement storage: equal-width movement is not restricted", "[traffic][movement]") {
    Graph g = make_diverge();
    auto m = make_ltm(g, true);
    for (AgentId a = 0; a < 26; ++a) { m.on_enter(0, a, 0.0); m.assign_movement(0, a, 1); }
    REQUIRE(m.has_capacity_for(0, 1));          // 26 < 26.67: share is 1 for e1
}

// End to end: 150 agents to the exit (bottleneck after the ramp) then 30
// through agents. Without movement storage the exit queue fills the approach
// and blocks the through agents; with it they pass at near free flow.
static double max_through_dwell_on_approach(bool movement_storage) {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_diverge(200.0f));
    SimulationConfig cfg;
    cfg.start_time = 0.0; cfg.end_time = 1800.0; cfg.sync_window_s = 5.0f;
    cfg.reroute_interval_s = 1e9f; cfg.num_threads = 1;
    cfg.stuck_threshold_ratio = 0.0f; cfg.stuck_max_hours = 0.0f;
    cfg.enable_pretrip_reroute = false;

    LtmTrafficModel::Config lc;
    lc.enable_discharge_cap = true;
    lc.discharge_burst_s    = 1.0f;
    lc.movement_storage     = movement_storage;

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph, lc));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od = fs::temp_directory_path() /
              ("nomad_test_od_diverge_" + std::to_string(movement_storage) + ".csv");
    {
        std::ofstream f(od);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,4,150,car,0,60\n";      // exit agents: ids 0..149
        f << "0,2,30,car,300,360\n";    // through agents: ids 150..179
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));

    std::unordered_map<AgentId, double> t_e0, t_e1;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        if (e.agent < 150) return;
        if (e.payload == 0) t_e0[e.agent] = e.time;
        if (e.payload == 1) t_e1[e.agent] = e.time;
    });
    sim.run();
    fs::remove(od);

    REQUIRE(t_e1.size() == 30);   // every through agent got through
    double worst = 0.0;   // longest time a through agent spent on the approach e0
    for (auto& [a, t1] : t_e1) worst = std::max(worst, t1 - t_e0.at(a));
    return worst;
}

TEST_CASE("Movement storage: through traffic is not blocked by an exit queue", "[traffic][movement][integration]") {
    double blocked = max_through_dwell_on_approach(false);
    double free    = max_through_dwell_on_approach(true);
    INFO("worst through dwell on e0: without=" << blocked << " s, with=" << free << " s");
    REQUIRE(free < 20.0);        // free flow on e0 is 10 s
    REQUIRE(blocked > 300.0);    // documents the diverge blocking it removes
}
