// QNode node model (SimulationConfig::node_model = "qnode"), after MATSim's
// QSim QNode. A car whose earliest exit time has passed joins the FIFO buffer
// of its link. Once per simulated second each node serves its in-links with a
// non-empty buffer in random order weighted by link capacity; each in-link
// releases vehicles in FIFO order while the traffic model admits them
// (downstream storage, discharge cap, stuck-time squeeze). With movement
// storage on, FIFO holds per movement (a blocked movement does not block the
// others on a multi-lane link).
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace nomad;

namespace {
EdgeData mk(NodeId t, float len, float cap) {
    EdgeData d{};
    d.target = t; d.length_m = len; d.free_flow_speed = 20.0f; d.capacity = cap;
    d.road_class = static_cast<uint8_t>(RoadClass::Primary);
    return d;
}

//  0 --e0 (1800/h)--.
//                    2 --e2 (100 m, 6000/h)--> 3 --e3 (900/h)--> 4
//  1 --e1 ( 600/h)--/
Graph make_merge() {
    Graph g;
    g.nodes.resize(5);
    const float xy[5][2] = {{0.0f, 0.01f}, {0.0f, -0.01f}, {0.01f, 0.0f}, {0.02f, 0.0f}, {0.03f, 0.0f}};
    for (int i = 0; i < 5; ++i) g.nodes[i] = {xy[i][0], xy[i][1], 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 1, 2, 3, 4, 4};
    g.col_idx = {0, 1, 2, 3};
    g.edges = {mk(2, 200.0f, 1800.0f), mk(2, 200.0f, 600.0f), mk(3, 100.0f, 6000.0f), mk(4, 100.0f, 900.0f)};
    g.geom_ptr.assign(5, 0);
    return g;
}

SimulationConfig base_cfg(const std::string& node_model) {
    SimulationConfig cfg;
    cfg.start_time = 0.0; cfg.end_time = 2400.0; cfg.sync_window_s = 5.0f;
    cfg.reroute_interval_s = 1e9f; cfg.num_threads = 1;
    cfg.stuck_threshold_ratio = 0.0f; cfg.stuck_max_hours = 0.0f;
    cfg.enable_pretrip_reroute = false;
    cfg.node_model = node_model;
    return cfg;
}

struct MergeResult {
    int from_a = 0, from_b = 0;              // entries into e2, 600 s <= t < 1800 s
    std::vector<AgentId> enter_e2, enter_e3; // entry order
};

MergeResult run_merge(const std::string& node_model) {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_merge());
    LtmTrafficModel::Config lc;
    lc.enable_discharge_cap = true;
    lc.discharge_burst_s    = 1.0f;
    lc.stuck_time_s         = 0.0f;
    Simulation sim(base_cfg(node_model));
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph, lc));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = fs::temp_directory_path() / ("nomad_test_qnode_merge_" + node_model + ".csv");
    {
        std::ofstream f(od);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,4,500,car,0,30\n";   // approach A: agents 0..499
        f << "1,4,500,car,0,30\n";   // approach B: agents 500..999
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    MergeResult r;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        if (e.payload == 2) {
            r.enter_e2.push_back(e.agent);
            if (e.time >= 600.0 && e.time < 1800.0) (e.agent < 500 ? r.from_a : r.from_b)++;
        }
        if (e.payload == 3) r.enter_e3.push_back(e.agent);
    });
    sim.run();
    fs::remove(od);
    return r;
}
} // namespace

TEST_CASE("QNode: a congested merge is shared in proportion to approach capacity",
          "[traffic][qnode][integration]") {
    auto r = run_merge("qnode");
    INFO("entries into e2 600-1800 s: A=" << r.from_a << " B=" << r.from_b);
    REQUIRE(r.from_b > 0);                       // the small approach is not starved
    const double ratio = static_cast<double>(r.from_a) / r.from_b;
    REQUIRE(ratio > 2.2);                        // capacity ratio 1800:600 = 3
    REQUIRE(ratio < 4.0);
    // total through the merge is set by the 900 veh/h bottleneck downstream
    REQUIRE(r.from_a + r.from_b > 0.85 * 900 * (1200.0 / 3600.0));
}

TEST_CASE("QNode: vehicles leave a link in the order they entered it",
          "[traffic][qnode][integration]") {
    auto r = run_merge("qnode");
    REQUIRE(r.enter_e3.size() > 100);
    std::vector<AgentId> prefix(r.enter_e2.begin(), r.enter_e2.begin() + r.enter_e3.size());
    REQUIRE(prefix == r.enter_e3);
}

TEST_CASE("QNode: identical runs give identical results", "[traffic][qnode][integration]") {
    auto r1 = run_merge("qnode");
    auto r2 = run_merge("qnode");
    REQUIRE(r1.enter_e2 == r2.enter_e2);
    REQUIRE(r1.enter_e3 == r2.enter_e3);
}

namespace {
// Diverge: 0 -e0 (approach, 4 lanes)-> 1; 1 -e1 (through)-> 2;
// 1 -e2 (1-lane exit ramp)-> 3 -e3 (60 veh/h bottleneck)-> 4.
Graph make_diverge() {
    Graph g;
    g.nodes.resize(5);
    for (int i = 0; i < 5; ++i) g.nodes[i] = {static_cast<float>(i) * 0.01f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.nodes[3].lat = -0.005f; g.nodes[4].lat = -0.01f;
    g.row_ptr = {0, 1, 3, 3, 4, 4};
    g.col_idx = {0, 1, 2, 3};
    g.edges = {mk(1, 200.0f, 6400.0f), mk(2, 200.0f, 6400.0f), mk(3, 100.0f, 1600.0f), mk(4, 100.0f, 60.0f)};
    g.geom_ptr.assign(5, 0);
    return g;
}

double worst_through_dwell(bool movement_storage) {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_diverge());
    LtmTrafficModel::Config lc;
    lc.enable_discharge_cap = true;
    lc.discharge_burst_s    = 1.0f;
    lc.movement_storage     = movement_storage;
    auto cfg = base_cfg("qnode");
    cfg.end_time = 1800.0;
    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph, lc));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = fs::temp_directory_path() /
              ("nomad_test_qnode_diverge_" + std::to_string(movement_storage) + ".csv");
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
    if (t_e1.size() != 30) return 1e9;
    double worst = 0.0;
    for (auto& [a, t1] : t_e1) worst = std::max(worst, t1 - t_e0.at(a));
    return worst;
}
} // namespace

TEST_CASE("QNode: with movement storage, FIFO holds per movement", "[traffic][qnode][integration]") {
    const double per_link = worst_through_dwell(false);
    const double per_mv   = worst_through_dwell(true);
    INFO("worst through dwell on e0: link FIFO=" << per_link << " s, movement FIFO=" << per_mv << " s");
    REQUIRE(per_mv < 20.0);          // free flow on e0 is 10 s (+ node step rounding)
    REQUIRE(per_link > 100.0);       // plain FIFO: the ramp queue blocks through cars
}

namespace {
// Diverge `chain` short links downstream of the observed approach e0:
// 0 -e0 (3 lanes, 300 m)-> 1 -(chain x 17 m, 4 lanes)-> D;
// D -ramp (1 lane)-> R -(60 veh/h bottleneck)-> X;  D -through (3 lanes)-> T.
// Edge ids: e0 = 0, chain = 1..chain, ramp, bottleneck, through.
struct Gore { Graph g; NodeId ramp_dest, through_dest; EdgeId first_chain; };
Gore make_gore(int chain) {
    Gore r;
    Graph& g = r.g;
    const int D = 1 + chain, R = D + 1, X = D + 2, T = D + 3, N = D + 4;
    g.nodes.resize(N);
    for (int i = 0; i < N; ++i) g.nodes[i] = {static_cast<float>(i) * 0.003f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.nodes[R].lat = -0.003f; g.nodes[X].lat = -0.006f;
    g.edges.push_back(mk(1, 300.0f, 4800.0f));                                // e0
    for (int k = 0; k < chain; ++k) g.edges.push_back(mk(2 + k, 17.0f, 6400.0f));
    const EdgeId ramp = g.edges.size();  g.edges.push_back(mk(R, 100.0f, 1600.0f));
    const EdgeId bott = g.edges.size();  g.edges.push_back(mk(X, 100.0f, 60.0f));
    const EdgeId thru = g.edges.size();  g.edges.push_back(mk(T, 200.0f, 4800.0f));
    // CSR: node i (< D) has out-edge i; D has ramp, through; R has bottleneck
    g.row_ptr.assign(N + 1, 0);
    g.col_idx.clear();
    for (int u = 0; u < N; ++u) {
        if (u < D) g.col_idx.push_back(u);
        else if (u == D) { g.col_idx.push_back(ramp); g.col_idx.push_back(thru); }
        else if (u == R) g.col_idx.push_back(bott);
        g.row_ptr[u + 1] = g.col_idx.size();
    }
    g.geom_ptr.assign(g.edges.size() + 1, 0);
    r.ramp_dest = X; r.through_dest = T; r.first_chain = 1;
    return r;
}

double worst_gore_through_dwell(bool lookahead, int chain) {
    namespace fs = std::filesystem;
    Gore gore = make_gore(chain);
    auto graph = std::make_shared<Graph>(gore.g);
    LtmTrafficModel::Config lc;
    lc.enable_discharge_cap = true;
    lc.discharge_burst_s    = 1.0f;
    lc.movement_storage     = true;
    lc.movement_lookahead   = lookahead;
    auto cfg = base_cfg("qnode");
    cfg.end_time = 1800.0;
    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph, lc));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = fs::temp_directory_path() /
              ("nomad_test_qnode_gore_" + std::to_string(lookahead) + "_" + std::to_string(chain) + ".csv");
    {
        std::ofstream f(od);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0," << gore.ramp_dest << ",150,car,0,60\n";      // ramp agents: ids 0..149
        f << "0," << gore.through_dest << ",30,car,300,360\n"; // through agents: ids 150..179
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    std::unordered_map<AgentId, double> t_e0, t_e1;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        if (e.agent < 150) return;
        if (e.payload == 0) t_e0[e.agent] = e.time;
        if (e.payload == gore.first_chain) t_e1[e.agent] = e.time;
    });
    sim.run();
    fs::remove(od);
    if (t_e1.size() != 30) return 1e9;   // some through vehicles never left e0
    double worst = 0.0;
    for (auto& [a, t1] : t_e1) worst = std::max(worst, t1 - t_e0.at(a));
    return worst;
}
} // namespace

TEST_CASE("QNode: with movement lookahead, a blocked ramp downstream does not block through traffic",
          "[traffic][qnode][integration]") {
    for (int chain : {1, 3}) {
        const double plain = worst_gore_through_dwell(false, chain);
        const double look  = worst_gore_through_dwell(true, chain);
        INFO("diverge " << chain << " link(s) downstream: worst through dwell on e0: movement FIFO="
             << plain << " s, with lookahead=" << look << " s");
        REQUIRE(look < 30.0);        // free flow on e0 is 15 s (+ node step rounding)
        REQUIRE(plain > 100.0);      // the ramp-bound head blocks through vehicles
    }
}

#include <nomad/config/scenario_config.hpp>

TEST_CASE("QNode: node_model is read from the scenario and validated", "[traffic][qnode][config]") {
    namespace fs = std::filesystem;
    auto p = fs::temp_directory_path() / "nomad_test_qnode_scenario.json";
    {
        std::ofstream f(p);
        f << R"({"simulation": {"start_time": 0, "end_time": 3600, "node_model": "qnode"},
                 "network": {"osm_pbf": "/dev/null"}})";
    }
    auto cfg = ScenarioConfigIO::load(p);
    REQUIRE(cfg.simulation.node_model == "qnode");
    // (other required fields are missing in this minimal file; only the
    // node_model check is under test)
    REQUIRE(ScenarioConfigIO::validate(cfg).find("node_model") == std::string::npos);
    cfg.simulation.node_model = "matsim";
    cfg.demand.source = "none";
    REQUIRE(ScenarioConfigIO::validate(cfg).find("node_model") != std::string::npos);
    fs::remove(p);
}

TEST_CASE("QNode: default node model is the per-agent retry model", "[traffic][qnode][config]") {
    REQUIRE(SimulationConfig{}.node_model == "agent");
}
