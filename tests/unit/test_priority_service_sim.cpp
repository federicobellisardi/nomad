// QNode with priority-ordered service: at a saturated merge the major approach
// is served first and the minor one gets the residual receiving capacity.
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <filesystem>
#include <fstream>

using namespace nomad;
namespace fs = std::filesystem;

namespace {
//   main 0 --e0 (primary)--------> 1 --e2 (primary, bottleneck)--> 3 --e3--> 4
//   ramp 2 --e1 (motorway_link)---^
Graph make_graph() {
    Graph g;
    g.nodes.resize(5);
    const float xy[5][2] = {{0.0f, 0.0f}, {0.01f, 0.0f}, {0.01f, -0.01f}, {0.02f, 0.0f}, {0.03f, 0.0f}};
    for (int i = 0; i < 5; ++i) g.nodes[i] = {xy[i][0], xy[i][1], 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 1, 2, 3, 4, 4};
    g.col_idx = {0, 2, 1, 3};
    auto mk = [](NodeId t, float len, RoadClass rc, float cap) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 15.0f; d.capacity = cap;
        d.road_class = static_cast<uint8_t>(rc);
        return d;
    };
    g.edges = {mk(1, 1000.0f, RoadClass::Primary, 1800.0f),
               mk(1, 1000.0f, RoadClass::MotorwayLink, 1800.0f),
               mk(3, 100.0f, RoadClass::Primary, 600.0f),       // bottleneck: 600 veh/h
               mk(4, 500.0f, RoadClass::Primary, 1800.0f)};
    g.geom_ptr.assign(5, 0);
    return g;
}

// Share of the bottleneck entries number skip+1 .. skip+n (once its storage
// has filled and the merge is saturated) that come from the ramp.
double ramp_share(bool service, int skip = 20, int first_n = 60) {
    auto graph = std::make_shared<Graph>(make_graph());
    SimulationConfig cfg;
    cfg.start_time = 0.0; cfg.end_time = 1800.0; cfg.sync_window_s = 5.0f;
    cfg.reroute_interval_s = 1e9f; cfg.num_threads = 1;
    cfg.stuck_threshold_ratio = 0.0f; cfg.stuck_max_hours = 0.0f;
    cfg.enable_pretrip_reroute = false;
    cfg.node_model = "qnode";
    Simulation sim(cfg);
    sim.set_graph(graph);
    LtmTrafficModel::Config lc;
    lc.enable_discharge_cap = true;
    lc.priority_service = service;
    sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph, lc));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    auto od = fs::temp_directory_path() / (std::string("nomad_test_prio_service_") + (service ? "on" : "off") + ".csv");
    {
        std::ofstream f(od);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,4,150,car,60,30\n";     // both approaches loaded far above 600 veh/h
        f << "2,4,150,car,60,30\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(OdMatrixDemand::from_csv(od)));
    int seen = 0, from_ramp = 0;
    std::vector<EdgeId> cur(1000, kInvalidEdge);
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        const EdgeId x = static_cast<EdgeId>(e.payload);
        if (e.agent < cur.size()) {
            if (x == 2) {
                ++seen;
                if (seen > skip && seen <= skip + first_n && cur[e.agent] == 1) ++from_ramp;
            }
            cur[e.agent] = x;
        }
    });
    sim.run();
    fs::remove(od);
    REQUIRE(seen >= skip + first_n);
    return static_cast<double>(from_ramp) / first_n;
}
} // namespace

TEST_CASE("QNode priority service: the major approach is served first at a saturated merge",
          "[traffic][priority][service][integration]") {
    const double off = ramp_share(false), on = ramp_share(true);
    INFO("ramp share of bottleneck entries 21-80: off " << off << ", on " << on);
    CHECK(off > 0.3);        // capacity-proportional: about half
    CHECK(on < 0.15);        // the ramp waits while the main road has queued cars
}
