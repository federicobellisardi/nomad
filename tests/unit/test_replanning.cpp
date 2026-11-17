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

#include <cmath>
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

struct RunOut {
    std::size_t kept = 0, rerouted = 0, mismatch = 0; std::vector<Plan> plans;
    double rel_gap = -1.0;
};

// gap_alpha > 0 switches on the gap-based rule (fraction is then ignored)
RunOut run(const std::string& tag, const fs::path& routes_in, float fraction, uint32_t seed,
           int count = 200, float nested_scale = 0.0f,
           float gap_alpha = 0.0f, float gap_band = 0.0f) {
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
        // nested sampling draws departures in [start, end): give it a real window
        f << "0,3," << count << ",car,100," << (nested_scale > 0.0f ? 400 : 30) << "\n";
    }
    if (nested_scale > 0.0f)
        sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(
            od, 42, -1e9f, 1e9f, {}, nested_scale, true)));
    else
        sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    auto out = fs::temp_directory_path() / ("nomad_test_replan_routes_" + tag + ".bin");
    sim.set_routes_out(out);
    if (!routes_in.empty()) sim.set_replanning(routes_in, fraction, seed);
    if (gap_alpha > 0.0f) sim.set_replan_gap(gap_alpha, gap_band);
    sim.run();
    RunOut r{sim.replan_kept(), sim.replan_rerouted(), sim.replan_mismatch(), read_plans(out),
             sim.replan_relative_gap()};
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

TEST_CASE("Replanning: with nested sampling, plans learnt at a lower scale are reused by trip",
          "[routing][replan][nested]") {
    // iteration at scale 0.5, every plan switched to the long path
    auto it0 = run("nested_it0", {}, 1.0f, 0, 400, 0.5f);
    for (auto& pl : it0.plans) { REQUIRE(pl.edges == kShort); pl.edges = kLong; }
    auto p = fs::temp_directory_path() / "nomad_test_replan_long_nested.bin";
    write_plans(p, it0.plans, 4);
    // scale 1.0, keep every matched plan
    auto r = run("nested_it1", p, 0.0f, 1, 400, 1.0f);
    fs::remove(p);
    INFO("scale 0.5 agents " << it0.plans.size() << ", scale 1.0 agents " << r.plans.size()
         << ": kept " << r.kept << " mismatch " << r.mismatch);
    REQUIRE(r.plans.size() == 400);
    REQUIRE(r.kept == it0.plans.size());              // every 0.5 trip found its plan
    REQUIRE(r.mismatch == 400 - it0.plans.size());     // the new trips are routed afresh
    REQUIRE(long_agents(r).size() == it0.plans.size());
}

// Gap-based replanning. On make_two_routes() at 20 m/s and free flow the long
// route costs 40 s and the short one 15 s: every stored long plan has a
// relative gap (40 - 15) / 40 = 0.625.
TEST_CASE("Replanning (gap rule): switch probability is alpha times the relative gap",
          "[routing][replan][gap]") {
    auto in = long_plans("gap_share");
    auto r = run("gap_share", in, 0.0f, 7, 200, 0.0f, 1.0f, 0.0f);
    REQUIRE(r.mismatch == 0);
    REQUIRE(r.kept + r.rerouted == 200);
    REQUIRE(r.rerouted > 100);           // binomial(200, 0.625): mean 125, sd 6.8
    REQUIRE(r.rerouted < 150);
    REQUIRE(long_agents(r).size() == r.kept);
    REQUIRE(std::abs(r.rel_gap - 0.625) < 1e-3);
    // alpha * gap >= 1: everybody switches
    auto all = run("gap_all", in, 0.0f, 7, 200, 0.0f, 2.0f, 0.0f);
    REQUIRE(all.rerouted == 200);
    REQUIRE(long_agents(all).empty());
    fs::remove(in);
}

TEST_CASE("Replanning (gap rule): gains inside the indifference band never switch",
          "[routing][replan][gap]") {
    auto in = long_plans("gap_band");
    auto r = run("gap_band", in, 1.0f, 7, 200, 0.0f, 2.0f, 0.7f);
    REQUIRE(r.kept == 200);
    REQUIRE(long_agents(r).size() == 200);
    REQUIRE(std::abs(r.rel_gap - 0.625) < 1e-3);   // the gap is still measured
    fs::remove(in);
}

TEST_CASE("Replanning (gap rule): agents already on the best route keep it, gap 0",
          "[routing][replan][gap]") {
    auto it0 = run("gap_opt_it0", {}, 1.0f, 0);
    auto p = fs::temp_directory_path() / "nomad_test_replan_short.bin";
    write_plans(p, it0.plans, 4);
    auto r = run("gap_opt", p, 1.0f, 7, 200, 0.0f, 5.0f, 0.0f);
    fs::remove(p);
    REQUIRE(r.kept == 200);
    REQUIRE(long_agents(r).empty());
    REQUIRE(std::abs(r.rel_gap) < 1e-6);
}

TEST_CASE("Replanning (gap rule): options are read from the scenario and validated",
          "[routing][replan][gap][config]") {
    auto p = fs::temp_directory_path() / "nomad_test_replan_gap_scenario.json";
    {
        std::ofstream f(p);
        f << R"({"simulation": {"start_time": 0, "end_time": 3600},
                 "network": {"osm_pbf": "/dev/null"},
                 "routing": {"routes_in": "a.bin", "replan_rule": "gap",
                             "replan_gap_alpha": 1.5, "replan_gap_band": 0.05},
                 "output": {"td_costs_out": "c.bin", "td_smoothing": 0.5}})";
    }
    auto cfg = ScenarioConfigIO::load(p);
    fs::remove(p);
    cfg.demand.source = "none";
    REQUIRE(cfg.routing.replan_rule == "gap");
    REQUIRE(cfg.routing.replan_gap_alpha == 1.5f);
    REQUIRE(cfg.routing.replan_gap_band == 0.05f);
    REQUIRE(cfg.output.td_smoothing == 0.5f);
    REQUIRE(ScenarioConfigIO::validate(cfg).empty());
    auto bad = cfg; bad.routing.replan_rule = "best";
    REQUIRE(ScenarioConfigIO::validate(bad).find("replan_rule") != std::string::npos);
    bad = cfg; bad.routing.replan_gap_alpha = 0.0f;
    REQUIRE(ScenarioConfigIO::validate(bad).find("replan_gap_alpha") != std::string::npos);
    bad = cfg; bad.output.td_smoothing = 0.0f;
    REQUIRE(ScenarioConfigIO::validate(bad).find("td_smoothing") != std::string::npos);
}
