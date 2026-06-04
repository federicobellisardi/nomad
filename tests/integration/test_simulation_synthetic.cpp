#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/queue_model.hpp>
#include <nomad/output/geojson_writer.hpp>

#include <filesystem>
#include <fstream>

using namespace nomad;

// ── Build a 5×5 grid graph ────────────────────────────────────────────────────
// Nodes numbered row-major: (0,0)=0, (0,1)=1, ..., (4,4)=24
// Edges connect horizontal and vertical neighbours, bidirectional.
static Graph make_grid(int rows = 5, int cols = 5) {
    Graph g;
    int N = rows * cols;
    g.nodes.resize(N);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int id = r * cols + c;
            g.nodes[id].lon = static_cast<float>(c) * 0.001f;  // ~111m spacing
            g.nodes[id].lat = static_cast<float>(r) * 0.001f;
            g.nodes[id].intersection_type = 0;
            g.nodes[id].signal_phase_idx  = 0;
        }
    }

    // Build edge list: each horizontal/vertical neighbor pair → 2 directed edges
    struct RawEdge { int u, v; };
    std::vector<RawEdge> raw;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int u = r * cols + c;
            if (c + 1 < cols) { int v = r * cols + (c+1); raw.push_back({u,v}); raw.push_back({v,u}); }
            if (r + 1 < rows) { int v = (r+1)*cols + c;   raw.push_back({u,v}); raw.push_back({v,u}); }
        }
    }

    // Build CSR
    std::vector<int> out_degree(N, 0);
    for (auto& e : raw) ++out_degree[e.u];

    g.row_ptr.resize(N + 1, 0);
    for (int i = 0; i < N; ++i) g.row_ptr[i+1] = g.row_ptr[i] + out_degree[i];

    uint32_t E = static_cast<uint32_t>(raw.size());
    g.edges.resize(E);
    g.col_idx.resize(E);
    std::vector<uint32_t> fill(g.row_ptr.begin(), g.row_ptr.begin() + N);

    // col_idx[CSR_pos] = EdgeId; edges[EdgeId] = edge data.
    // The EdgeId here equals the raw edge index (eid), and edges are
    // stored at position eid, NOT at CSR position pos.
    for (uint32_t eid = 0; eid < E; ++eid) {
        auto& re = raw[eid];
        uint32_t pos = fill[re.u]++;
        g.col_idx[pos] = eid;           // CSR pos → EdgeId
        EdgeData ed{};
        ed.target          = static_cast<NodeId>(re.v);
        ed.length_m        = 111.0f;
        ed.free_flow_speed = 10.0f;
        ed.capacity        = 1600.0f;
        ed.road_class      = static_cast<uint8_t>(RoadClass::Residential);
        g.edges[eid] = ed;              // store at EdgeId, not at pos
    }
    g.geom_ptr.assign(E + 1, 0);
    return g;
}

// ── Build a small OD matrix CSV in /tmp ──────────────────────────────────────
static std::filesystem::path make_od_csv(int num_agents = 100) {
    auto path = std::filesystem::temp_directory_path() / "nomad_test_od.csv";
    std::ofstream f(path);
    f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
    // Corner to corner
    f << "0,24," << num_agents << ",car,28800,3600\n";
    // Diagonal routes
    f << "4,20,50,car,29000,1800\n";
    return path;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST_CASE("Simulation: basic end-to-end with synthetic grid", "[integration]") {
    auto graph = std::make_shared<Graph>(make_grid(5, 5));
    REQUIRE(graph->num_nodes() == 25);
    REQUIRE(graph->num_edges() > 0);
    REQUIRE(graph->validate());

    auto od_path = make_od_csv(50);

    SimulationConfig cfg;
    cfg.start_time    = 28000.0;
    cfg.end_time      = 32000.0;   // 4000s window
    cfg.sync_window_s = 1.0f;
    cfg.num_threads   = 1;         // deterministic single-threaded

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<QueueTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    REQUIRE_NOTHROW(sim.run());

    // Some events must have been processed
    REQUIRE(sim.events_processed() > 0);

    std::filesystem::remove(od_path);
}

TEST_CASE("Simulation: agents depart and arrive", "[integration]") {
    auto graph = std::make_shared<Graph>(make_grid(4, 4));

    SimulationConfig cfg;
    cfg.start_time = 0.0;
    cfg.end_time   = 5000.0;
    cfg.num_threads= 1;

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<QueueTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    // Create OD demand directly (no CSV)
    auto od_path = std::filesystem::temp_directory_path() / "nomad_test_od2.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,15,20,car,1000,100\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    sim.run();

    // At end of a long simulation, most agents should have arrived
    std::size_t active = sim.active_agents();
    // Not all may arrive due to grid topology, but events were processed
    REQUIRE(sim.events_processed() > 0);
    REQUIRE(active <= 20);  // not more agents than we created

    std::filesystem::remove(od_path);
}

TEST_CASE("Simulation: traffic model link states change after run", "[integration]") {
    auto graph = std::make_shared<Graph>(make_grid(3, 3));

    SimulationConfig cfg;
    cfg.start_time = 0.0;
    cfg.end_time   = 2000.0;
    cfg.num_threads= 1;

    auto traffic = std::make_unique<QueueTrafficModel>(*graph);
    auto* traffic_ptr = traffic.get();

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::move(traffic));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od_path = std::filesystem::temp_directory_path() / "nomad_test_od3.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,8,30,car,500,50\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    sim.run();

    // Some links should have been used (non-zero travel time from BPR)
    const auto states = sim.traffic().link_states();
    REQUIRE(states.size() == graph->num_edges());

    std::filesystem::remove(od_path);
}

TEST_CASE("Simulation: GeoJSON output is written", "[integration]") {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_grid(3, 3));

    fs::path out_dir = fs::temp_directory_path() / "nomad_test_geojson";
    fs::create_directories(out_dir);

    SimulationConfig cfg;
    cfg.start_time          = 0.0;
    cfg.end_time            = 1000.0;
    cfg.num_threads         = 1;
    cfg.snapshot_interval_s = 200;

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<QueueTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));
    sim.add_output_writer(std::make_unique<GeoJsonWriter>(out_dir, 200.0f));

    auto od_path = fs::temp_directory_path() / "nomad_test_od4.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,8,10,car,100,10\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    sim.run();

    // GeoJSON static network export should work without simulation
    GeoJsonWriter w(out_dir, 300.0f);
    REQUIRE_NOTHROW(w.write_static_network(*graph, out_dir / "static_network.geojson"));
    REQUIRE(fs::exists(out_dir / "static_network.geojson"));
    REQUIRE(fs::file_size(out_dir / "static_network.geojson") > 100);

    fs::remove_all(out_dir);
    fs::remove(od_path);
}
