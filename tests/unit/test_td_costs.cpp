// Day-to-day assignment support: time-dependent link costs (TdCostTable),
// experienced travel-time recording (TdCostRecorder / Simulation) and
// time-dependent A* pre-routing.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/routing/td_costs.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>

using namespace nomad;
using Catch::Matchers::WithinAbs;
namespace fs = std::filesystem;

// Diamond: 0 --e0 (100 m)--> 1 --e2 (100 m)--> 3   (short path A)
//          0 --e1 (150 m)--> 2 --e3 (150 m)--> 3   (long path B)
static Graph make_diamond() {
    Graph g;
    g.nodes.resize(4);
    for (int i = 0; i < 4; ++i) g.nodes[i] = {static_cast<float>(i) * 0.001f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 2, 3, 4, 4};
    g.col_idx = {0, 1, 2, 3};
    auto mk = [](NodeId t, float len) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(RoadClass::Primary);
        return d;
    };
    g.edges = {mk(1, 100.0f), mk(2, 150.0f), mk(3, 100.0f), mk(3, 150.0f)};
    g.geom_ptr.assign(5, 0);
    return g;
}

TEST_CASE("TdCostTable: bins, free-flow fallback and floor", "[routing][td]") {
    TdCostTable t(2, 3, 900.0f, 0.0);
    t.tt()[0 * 3 + 1] = 50.0f;     // e0, bin 1
    t.tt()[0 * 3 + 2] = 5.0f;      // e0, bin 2: below free flow
    CHECK(t.cost(0, 100.0, 10.0f)  == 10.0f);   // bin 0: no data -> ff
    CHECK(t.cost(0, 1000.0, 10.0f) == 50.0f);   // bin 1
    CHECK(t.cost(0, 2000.0, 10.0f) == 10.0f);   // floor at ff
    CHECK(t.cost(0, 1e6, 10.0f)    == 10.0f);   // past the end -> last bin
    CHECK(t.cost(0, -50.0, 10.0f)  == 10.0f);   // before t0 -> bin 0
    CHECK(t.cost(7, 1000.0, 10.0f) == 10.0f);   // unknown edge -> ff
}

TEST_CASE("TdCostRecorder: mean per entry bin, save/load round trip", "[routing][td]") {
    TdCostRecorder rec(2, 0.0, 2700.0, 900.0f);
    rec.record(1, 100.0, 130.0);
    rec.record(1, 800.0, 850.0);    // same bin (entry time decides)
    rec.record(1, 950.0, 2000.0);   // bin 1
    rec.record(0, 50.0, 40.0);      // invalid (leave < enter): ignored
    TdCostTable t = rec.table();
    REQUIRE(t.num_bins() == 3);
    CHECK_THAT(t.tt()[1 * 3 + 0], WithinAbs(40.0, 1e-4));
    CHECK(t.n()[1 * 3 + 0] == 2);
    CHECK_THAT(t.tt()[1 * 3 + 1], WithinAbs(1050.0, 1e-3));
    CHECK(std::isnan(t.tt()[0]));

    auto p = fs::temp_directory_path() / "nomad_test_td_roundtrip.bin";
    t.save(p);
    TdCostTable u = TdCostTable::load(p);
    fs::remove(p);
    CHECK(u.num_edges() == 2);
    CHECK(u.bin_s() == 900.0f);
    CHECK(u.n() == t.n());
    CHECK_THAT(u.tt()[1 * 3 + 1], WithinAbs(1050.0, 1e-3));
}

TEST_CASE("Time-dependent A*: costs depend on the departure time", "[routing][td]") {
    Graph g = make_diamond();
    AStarRouter r(g, nullptr);
    TdCostTable t(4, 2, 900.0f, 0.0);
    t.tt()[0 * 2 + 1] = 500.0f;     // e0 congested only in bin 1 (900-1800 s)

    Route early = r.route_stochastic(0, 3, AgentMode::Car, 1u, 0.0f, &t, 0.0);
    Route late  = r.route_stochastic(0, 3, AgentMode::Car, 1u, 0.0f, &t, 1000.0);
    Route none  = r.route_stochastic(0, 3, AgentMode::Car, 1u, 0.0f, nullptr, 1000.0);
    REQUIRE(early.is_valid); REQUIRE(late.is_valid); REQUIRE(none.is_valid);
    CHECK(early.edges == std::vector<EdgeId>{0, 2});
    CHECK(late.edges  == std::vector<EdgeId>{1, 3});
    CHECK(none.edges  == std::vector<EdgeId>{0, 2});
    // The expected entry time of e2 (departure + 10 s on e0) picks its bin.
    t.tt()[2 * 2 + 1] = 500.0f;
    t.tt()[0 * 2 + 1] = std::nanf("");
    Route before = r.route_stochastic(0, 3, AgentMode::Car, 1u, 0.0f, &t, 880.0);  // e2 at 890 s, bin 0
    Route after  = r.route_stochastic(0, 3, AgentMode::Car, 1u, 0.0f, &t, 895.0);  // e2 at 905 s, bin 1
    CHECK(before.edges == std::vector<EdgeId>{0, 2});
    CHECK(after.edges  == std::vector<EdgeId>{1, 3});
}

static SimulationConfig small_cfg() {
    SimulationConfig cfg;
    cfg.start_time = 0.0; cfg.end_time = 900.0; cfg.sync_window_s = 5.0f;
    cfg.reroute_interval_s = 1e9f; cfg.num_threads = 1;
    cfg.stuck_threshold_ratio = 0.0f; cfg.stuck_max_hours = 0.0f;
    cfg.enable_pretrip_reroute = false;
    return cfg;
}

static fs::path write_od(const std::string& tag) {
    auto od = fs::temp_directory_path() / ("nomad_test_td_od_" + tag + ".csv");
    std::ofstream f(od);
    f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
    f << "0,3,20,car,0,60\n";
    return od;
}

TEST_CASE("Simulation records experienced link times by entry bin", "[routing][td][integration]") {
    auto graph = std::make_shared<Graph>(make_diamond());
    Simulation sim(small_cfg());
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = write_od("rec");
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    sim.enable_td_recording(300.0f);
    sim.run();
    fs::remove(od);

    auto p = fs::temp_directory_path() / "nomad_test_td_rec.bin";
    sim.write_td_costs(p);
    TdCostTable t = TdCostTable::load(p);
    fs::remove(p);
    REQUIRE(t.num_bins() == 3);
    uint32_t n0 = 0, n2 = 0, n1 = 0;
    for (uint32_t b = 0; b < 3; ++b) {
        n0 += t.n()[0 * 3 + b]; n1 += t.n()[1 * 3 + b]; n2 += t.n()[2 * 3 + b];
        for (EdgeId e : {0u, 2u}) {
            float v = t.tt()[e * 3 + b];
            if (t.n()[e * 3 + b] > 0) CHECK(v >= graph->free_flow_time(e) - 1e-3f);
        }
    }
    CHECK(n0 == 20);      // every car took the short path, once
    CHECK(n2 == 20);
    CHECK(n1 == 0);
}

TEST_CASE("Simulation pre-routes on time-dependent costs", "[routing][td][integration]") {
    auto graph = std::make_shared<Graph>(make_diamond());
    auto td = std::make_shared<TdCostTable>(4, 1, 900.0f, 0.0);
    td->tt()[0] = 1000.0f;            // e0 very slow all day
    Simulation sim(small_cfg());
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = write_od("pre");
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    sim.set_td_costs(td);
    std::set<EdgeId> used;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        used.insert(static_cast<EdgeId>(e.payload));
    });
    sim.run();
    fs::remove(od);
    CHECK(used.count(0) == 0);
    CHECK(used.count(1) == 1);
    CHECK(used.count(3) == 1);
}
