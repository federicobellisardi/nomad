// Partial replanning (Simulation::set_replanning / set_routes_out): route
// plans are written after pre-routing and read back in the next iteration,
// where only a drawn share of the agents is routed afresh.
#include <catch2/catch_test_macros.hpp>

#include <nomad/config/scenario_config.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

using namespace nomad;
namespace fs = std::filesystem;

namespace {
EdgeData mk(NodeId t, float len) {
    EdgeData d{};
    d.target = t; d.length_m = len; d.free_flow_speed = 20.0f; d.capacity = 1800.0f;
    d.road_class = static_cast<uint8_t>(RoadClass::Primary);
    return d;
}

// Two routes 0 -> 3: short via node 1 (e0, e2), long via node 2 (e1, e3).
Graph make_two_routes() {
    Graph g;
    g.nodes.resize(4);
    const float xy[4][2] = {{0.0f, 0.0f}, {0.0f, 0.001f}, {0.0f, -0.003f}, {0.002f, 0.0f}};
    for (int i = 0; i < 4; ++i) g.nodes[i] = {xy[i][0], xy[i][1], 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 2, 3, 4, 4};
    g.col_idx = {0, 1, 2, 3};
    g.edges = {mk(1, 150.0f), mk(2, 400.0f), mk(3, 150.0f), mk(3, 400.0f)};
    g.geom_ptr.assign(5, 0);
    return g;
}

struct Plan { uint32_t o, d; double t; uint32_t m; std::vector<uint32_t> edges; };

std::vector<Plan> read_plans(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    char magic[4]; uint32_t ver, ne, na;
    f.read(magic, 4); f.read((char*)&ver, 4); f.read((char*)&ne, 4); f.read((char*)&na, 4);
    REQUIRE(std::memcmp(magic, "NRTE", 4) == 0);
    std::vector<Plan> out(na);
    for (auto& pl : out) {
        uint32_t len;
        f.read((char*)&pl.o, 4); f.read((char*)&pl.d, 4); f.read((char*)&pl.t, 8);
        f.read((char*)&pl.m, 4); f.read((char*)&len, 4);
        pl.edges.resize(len); f.read((char*)pl.edges.data(), 4 * len);
    }
    REQUIRE(f.good());
    return out;
}

void write_plans(const fs::path& p, const std::vector<Plan>& plans, uint32_t num_edges) {
    std::ofstream f(p, std::ios::binary);
    const uint32_t ver = 1, na = static_cast<uint32_t>(plans.size());
    f.write("NRTE", 4); f.write((const char*)&ver, 4); f.write((const char*)&num_edges, 4);
    f.write((const char*)&na, 4);
    for (auto& pl : plans) {
        const uint32_t len = static_cast<uint32_t>(pl.edges.size());
        f.write((const char*)&pl.o, 4); f.write((const char*)&pl.d, 4); f.write((const char*)&pl.t, 8);
        f.write((const char*)&pl.m, 4); f.write((const char*)&len, 4);
        f.write((const char*)pl.edges.data(), 4 * len);
    }
}

struct RunOut { std::size_t kept = 0, rerouted = 0, mismatch = 0; std::vector<Plan> plans; };

RunOut run(const std::string& tag, const fs::path& routes_in, float fraction, uint32_t seed,
           int count = 200) {
    auto graph = std::make_shared<Graph>(make_two_routes());
    SimulationConfig cfg;
    cfg.start_time = 0.0; cfg.end_time = 600.0; cfg.sync_window_s = 5.0f;
    cfg.reroute_interval_s = 1e9f; cfg.num_threads = 1;
    cfg.stuck_threshold_ratio = 0.0f; cfg.stuck_max_hours = 0.0f;
    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = fs::temp_directory_path() / ("nomad_test_replan_od_" + tag + ".csv");
    {
        std::ofstream f(od);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,3," << count << ",car,100,30\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    auto out = fs::temp_directory_path() / ("nomad_test_replan_routes_" + tag + ".bin");
    sim.set_routes_out(out);
    if (!routes_in.empty()) sim.set_replanning(routes_in, fraction, seed);
    sim.run();
    RunOut r{sim.replan_kept(), sim.replan_rerouted(), sim.replan_mismatch(), read_plans(out)};
    fs::remove(od); fs::remove(out);
    return r;
}

const std::vector<uint32_t> kShort = {0, 2}, kLong = {1, 3};

// Iteration-0 plans with every route switched to the long path, so that a
// kept plan (long) is distinguishable from a fresh one (short).
fs::path long_plans(const std::string& tag) {
    auto it0 = run(tag + "_it0", {}, 1.0f, 0);
    for (auto& pl : it0.plans) { REQUIRE(pl.edges == kShort); pl.edges = kLong; }
    auto p = fs::temp_directory_path() / ("nomad_test_replan_long_" + tag + ".bin");
    write_plans(p, it0.plans, 4);
    return p;
}

std::set<std::size_t> long_agents(const RunOut& r) {
    std::set<std::size_t> s;
    for (std::size_t a = 0; a < r.plans.size(); ++a) if (r.plans[a].edges == kLong) s.insert(a);
    return s;
}
} // namespace

TEST_CASE("Replanning: only the drawn share of agents is routed afresh", "[routing][replan]") {
    auto in = long_plans("share");
    auto r = run("share", in, 0.3f, 7);
    REQUIRE(r.mismatch == 0);
    REQUIRE(r.kept + r.rerouted == 200);
    REQUIRE(r.rerouted > 35);            // binomial(200, 0.3): mean 60, sd 6.5
    REQUIRE(r.rerouted < 85);
    // kept agents follow their stored (long) route, the others the short one
    REQUIRE(long_agents(r).size() == r.kept);
    fs::remove(in);
}

TEST_CASE("Replanning: fraction 0 keeps every plan, fraction 1 re-routes all", "[routing][replan]") {
    auto in = long_plans("bounds");
    auto r0 = run("b0", in, 0.0f, 1);
    REQUIRE(r0.kept == 200);
    REQUIRE(long_agents(r0).size() == 200);
    auto r1 = run("b1", in, 1.0f, 1);
    REQUIRE(r1.rerouted == 200);
    REQUIRE(long_agents(r1).empty());
    fs::remove(in);
}

TEST_CASE("Replanning: the draw is reproducible and changes with the seed", "[routing][replan]") {
    auto in = long_plans("seed");
    auto a = long_agents(run("s1", in, 0.5f, 11));
    auto b = long_agents(run("s2", in, 0.5f, 11));
    auto c = long_agents(run("s3", in, 0.5f, 12));
    REQUIRE(a == b);
    REQUIRE(a != c);
    fs::remove(in);
}

TEST_CASE("Replanning: plans that no longer match the trip are re-routed", "[routing][replan]") {
    auto in = long_plans("mm");
    // fewer agents: ids 0..149 still match, the plan file has 200 entries
    auto r = run("mm", in, 0.0f, 1, 150);
    REQUIRE(r.kept + r.mismatch == 150);
    // a broken route (not connected) is never reused
    auto plans = read_plans(in);
    for (auto& pl : plans) pl.edges = {0, 3};
    write_plans(in, plans, 4);
    auto rb = run("mm_broken", in, 0.0f, 1);
    REQUIRE(rb.mismatch == 200);
    REQUIRE(long_agents(rb).empty());
    fs::remove(in);
}

TEST_CASE("Replanning: options are read from the scenario and validated", "[routing][replan][config]") {
    auto p = fs::temp_directory_path() / "nomad_test_replan_scenario.json";
    {
        std::ofstream f(p);
        f << R"({"simulation": {"start_time": 0, "end_time": 3600},
                 "network": {"osm_pbf": "/dev/null"},
                 "routing": {"routes_in": "a.bin", "replan_fraction": 0.1, "replan_seed": 3},
                 "output": {"routes_out": "b.bin"}})";
    }
    auto cfg = ScenarioConfigIO::load(p);
    REQUIRE(cfg.routing.routes_in == "a.bin");
    REQUIRE(cfg.routing.replan_fraction == 0.1f);
    REQUIRE(cfg.routing.replan_seed == 3u);
    REQUIRE(cfg.output.routes_out == "b.bin");
    REQUIRE(ScenarioConfigIO::validate(cfg).find("replan_fraction") == std::string::npos);
    cfg.routing.replan_fraction = 1.5f;
    cfg.demand.source = "none";
    REQUIRE(ScenarioConfigIO::validate(cfg).find("replan_fraction") != std::string::npos);
    fs::remove(p);
}
