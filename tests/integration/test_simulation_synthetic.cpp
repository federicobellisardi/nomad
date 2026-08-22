#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/traffic/queue_model.hpp>
#include <nomad/traffic/ltm_model.hpp>
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

// ── Multimodal: walk agents must not congest the shared car-calibrated link ──
// 2-node graph, single directed edge, deliberately low capacity so a naive
// (unfiltered) traffic-model interaction would push occupancy well past
// storage_cap and inflate BPR travel time. If handle_enter_link/handle_exit_link
// correctly gate traffic_->on_enter/on_exit to AgentMode::Car, the lone car
// agent still gets its free-flow travel time regardless of how many walk
// agents share the edge.
static Graph make_single_edge_low_cap() {
    Graph g;
    g.nodes.resize(2);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.row_ptr = {0, 1, 1};
    g.col_idx = {0};
    g.edges.resize(1);
    EdgeData ed{};
    ed.target          = 1;
    ed.length_m        = 1000.0f;
    ed.free_flow_speed = 10.0f;   // 100s free-flow
    ed.capacity        = 200.0f;  // low capacity → low storage_cap
    ed.road_class      = static_cast<uint8_t>(RoadClass::Residential);
    g.edges[0] = ed;
    g.geom_ptr.assign(2, 0);
    return g;
}

TEST_CASE("Simulation: walk agents do not congest a shared car link", "[integration][multimodal]") {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_single_edge_low_cap());

    SimulationConfig cfg;
    cfg.start_time    = 0.0;
    cfg.end_time      = 5000.0;
    cfg.sync_window_s = 1.0f;
    cfg.num_threads   = 1;

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::make_unique<QueueTrafficModel>(*graph));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od_path = fs::temp_directory_path() / "nomad_test_od_multimodal.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        f << "0,1,1,car,0,1\n";
        f << "0,1,30,walk,0,1\n";  // enough to blow past storage_cap if ungated
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    sim.run();

    const auto& hot = sim.agent_hot();
    bool found_car = false;
    for (std::size_t a = 0; a < hot.size(); ++a) {
        if (hot.mode[a] != AgentMode::Car) continue;
        found_car = true;
        float travel_s = static_cast<float>(hot.scheduled_exit[a] - hot.enter_time[a]);
        // Free-flow is 100s; congested (30 walk agents wrongly counted) would
        // be well over that (BPR caps at 3.4x = 340s for a fully-loaded link).
        REQUIRE(travel_s < 150.0f);
    }
    REQUIRE(found_car);

    fs::remove(od_path);
}

// ── Single edge with small storage_veh, for the departure-gate test below ────
static Graph make_small_storage_edge() {
    Graph g;
    g.nodes.resize(2);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.row_ptr = {0, 1, 1};
    g.col_idx = {0};
    g.edges.resize(1);
    EdgeData ed{};
    ed.target          = 1;
    ed.length_m        = 50.0f;   // effective-length floor itself
    ed.free_flow_speed = 10.0f;   // 5s free-flow
    ed.capacity        = 100.0f;  // 1 lane -> storage_veh = 50*1*(1/7.5) ≈ 6.67
    ed.road_class      = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[0] = ed;
    g.geom_ptr.assign(2, 0);
    return g;
}

TEST_CASE("Simulation: LTM departure gate keeps origin-edge occupancy within storage_veh",
          "[integration]") {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_small_storage_edge());

    SimulationConfig cfg;
    cfg.start_time    = 900.0;
    cfg.end_time      = 1200.0;
    cfg.sync_window_s = 1.0f;
    cfg.num_threads   = 1;
    // Isolate the departure gate: disable stuck-agent teleportation so it
    // can't confound the occupancy trace with an unrelated mechanism.
    cfg.stuck_threshold_ratio = 0.0f;
    cfg.stuck_max_hours       = 0.0f;

    auto traffic = std::make_unique<LtmTrafficModel>(*graph);
    auto* traffic_ptr = traffic.get();

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::move(traffic));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od_path = fs::temp_directory_path() / "nomad_test_od_departure_gate.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        // 40 agents clustered in a ~12s window onto an edge whose free-flow
        // transit time is only 5s -- before the departure-gate fix, all of
        // them enter unconditionally via handle_depart(), pushing occupancy
        // far past storage_veh (≈6.67). After the fix, has_capacity() defers
        // departures 1s at a time until the edge drains.
        f << "0,1,40,car,1000,2\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    float max_occupancy = 0.0f;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        if (static_cast<EdgeId>(e.payload) != 0) return;
        max_occupancy = std::max(max_occupancy,
                                  traffic_ptr->link_states()[0].occupancy.load());
    });

    sim.run();

    REQUIRE(max_occupancy <= 7.0f);  // storage_veh ≈ 6.67 -> 7 agents saturate it
                                      // (same threshold as the LTM unit tests above)

    fs::remove(od_path);
}

// ── N single-hop upstream edges converging on one shared downstream edge ────
// (nodes 0..n_upstream-1) --edge i--> (node "middle") --shared edge--> (node "final")
static Graph make_converging_star(int n_upstream) {
    int middle = n_upstream;
    int final_node = n_upstream + 1;
    int N = n_upstream + 2;

    struct RawEdge { int u, v; };
    std::vector<RawEdge> raw;
    for (int i = 0; i < n_upstream; ++i) raw.push_back({i, middle});
    raw.push_back({middle, final_node});  // the shared chokepoint, last in raw order

    Graph g;
    g.nodes.resize(N);
    for (int i = 0; i < N; ++i)
        g.nodes[i] = {static_cast<float>(i) * 0.001f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};

    std::vector<int> out_degree(N, 0);
    for (auto& e : raw) ++out_degree[e.u];
    g.row_ptr.assign(N + 1, 0);
    for (int i = 0; i < N; ++i) g.row_ptr[i + 1] = g.row_ptr[i] + out_degree[i];

    uint32_t E = static_cast<uint32_t>(raw.size());
    g.edges.resize(E);
    g.col_idx.resize(E);
    std::vector<uint32_t> fill(g.row_ptr.begin(), g.row_ptr.begin() + N);
    for (uint32_t eid = 0; eid < E; ++eid) {
        auto& re = raw[eid];
        uint32_t pos = fill[re.u]++;
        g.col_idx[pos] = eid;
        EdgeData ed{};
        ed.target = static_cast<NodeId>(re.v);
        ed.road_class = static_cast<uint8_t>(RoadClass::Primary);
        if (re.u == middle) {
            // The shared downstream chokepoint: same small storage_veh as
            // make_small_storage_edge (≈6.67) so n_upstream agents arriving
            // at once genuinely contend for it.
            ed.length_m = 50.0f;
            ed.free_flow_speed = 10.0f;
            ed.capacity = 100.0f;
        } else {
            // Upstream edges: generous capacity (only 1 agent each), but the
            // SAME 10s free-flow time so all n_upstream agents reach `middle`
            // and attempt to exit onto the shared edge in the same instant.
            ed.length_m = 100.0f;
            ed.free_flow_speed = 10.0f;
            ed.capacity = 1600.0f;
        }
        g.edges[eid] = ed;
    }
    g.geom_ptr.assign(E + 1, 0);
    return g;
}

TEST_CASE("Simulation: exit-link gate keeps shared downstream edge within storage_veh "
          "when multiple upstream agents converge simultaneously", "[integration]") {
    namespace fs = std::filesystem;
    const int n_upstream = 10;
    auto graph = std::make_shared<Graph>(make_converging_star(n_upstream));
    const EdgeId shared_edge = static_cast<EdgeId>(n_upstream);  // last edge built above

    SimulationConfig cfg;
    cfg.start_time    = 0.0;
    cfg.end_time      = 60.0;
    cfg.sync_window_s = 30.0f;  // one 10-agent origin edge each, all converging within
                                // this single batch is exactly the scenario that broke
                                // has_capacity() before enter_edge_now() was synchronous.
    cfg.num_threads   = 1;
    cfg.stuck_threshold_ratio = 0.0f;  // isolate the gate, see the departure-gate test above
    cfg.stuck_max_hours       = 0.0f;

    auto traffic = std::make_unique<LtmTrafficModel>(*graph);
    auto* traffic_ptr = traffic.get();

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::move(traffic));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od_path = fs::temp_directory_path() / "nomad_test_od_converging_star.csv";
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        // One agent per distinct origin node, all departing in [0,1) -- each
        // takes the SAME 10s free-flow hop, so all n_upstream agents try to
        // exit onto the shared edge within the same ~10-11s instant.
        for (int i = 0; i < n_upstream; ++i)
            f << i << "," << (n_upstream + 1) << ",1,car,0,1\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    float max_occupancy = 0.0f;
    sim.register_hook(EventType::AgentEnterLink, [&](const Event& e) {
        if (static_cast<EdgeId>(e.payload) != shared_edge) return;
        max_occupancy = std::max(max_occupancy,
                                  traffic_ptr->link_states()[shared_edge].occupancy.load());
    });

    sim.run();

    REQUIRE(max_occupancy <= 7.0f);  // storage_veh ≈ 6.67 on the shared edge

    fs::remove(od_path);
}

// ── Diamond: two parallel 0→3 paths of very different character ─────────────
// Fast-but-fragile: 0--A-->1--B-->3, short (5s ff each), low capacity
// (storage_veh ≈ 6.67, same floor as make_small_storage_edge) -- always the
// free-flow-optimal choice, so pre-routing (before any traffic exists)
// always picks it for every agent regardless of what happens later.
// Slow-but-safe: 0--C-->2--D-->3, long (50s ff each), generous capacity --
// never saturates, but is a worse choice under free-flow costs alone.
// Edge IDs are deterministic by build order: A=0 (0→1), B=1 (1→3), C=2
// (0→2), D=3 (2→3).
static Graph make_diamond() {
    Graph g;
    g.nodes.resize(4);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {0.0f, 1.0f, 0, 0, 0, 2};
    g.nodes[3] = {1.0f, 1.0f, 0, 0, 0, 3};
    // node 0 has 2 out-edges (A, C); node 1 has 1 (B); node 2 has 1 (D); node 3 has 0.
    g.row_ptr = {0, 2, 3, 4, 4};
    g.col_idx = {0, 2, 1, 3};  // CSR pos -> EdgeId, per node's out-edges above
    g.edges.resize(4);

    EdgeData a{};
    a.target = 1; a.length_m = 50.0f; a.free_flow_speed = 10.0f; a.capacity = 5.0f;
    a.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[0] = a;

    EdgeData b{};
    b.target = 3; b.length_m = 50.0f; b.free_flow_speed = 10.0f; b.capacity = 1600.0f;
    b.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[1] = b;

    EdgeData c{};
    c.target = 2; c.length_m = 500.0f; c.free_flow_speed = 10.0f; c.capacity = 10000.0f;
    c.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[2] = c;

    EdgeData d{};
    d.target = 3; d.length_m = 500.0f; d.free_flow_speed = 10.0f; d.capacity = 10000.0f;
    d.road_class = static_cast<uint8_t>(RoadClass::Primary);
    g.edges[3] = d;

    g.geom_ptr.assign(5, 0);
    return g;
}

// Runs the diamond scenario once with the given enable_pretrip_reroute
// setting; returns the first edge of the STORED route for the late agent
// (id 100) after the run. Edge A is deliberately congested heavily enough
// that agent 100 (departing well after 100 other agents already competing
// for the same tiny storage_veh) may never actually win a has_capacity()
// slot within this test's runtime -- checking the stored route directly
// (rather than waiting for an actual AgentEnterLink) sidesteps that
// entirely, and is safe here specifically because an agent that hasn't
// entered any edge yet cannot have been mid-trip-rerouted (that path only
// ever fires for AgentState::OnLink agents).
static EdgeId run_diamond_and_capture_late_agent_first_edge(bool enable_pretrip_reroute) {
    namespace fs = std::filesystem;
    auto graph = std::make_shared<Graph>(make_diamond());

    SimulationConfig cfg;
    cfg.start_time            = 0.0;
    cfg.end_time              = 900.0;
    cfg.sync_window_s         = 5.0f;
    cfg.reroute_interval_s    = 10.0f;
    cfg.num_threads           = 1;
    cfg.stuck_threshold_ratio = 0.0f;  // isolate the routing mechanism, not the teleport one
    cfg.stuck_max_hours       = 0.0f;
    cfg.enable_pretrip_reroute = enable_pretrip_reroute;

    LtmTrafficModel::Config lcfg;
    lcfg.enable_discharge_cap = true;
    lcfg.discharge_burst_s    = 1.0f;  // tight: max_credit floors to 1.0
    auto traffic = std::make_unique<LtmTrafficModel>(*graph, lcfg);

    Simulation sim(cfg);
    sim.set_graph(graph);
    sim.set_traffic_model(std::move(traffic));
    sim.set_router(std::make_unique<AStarRouter>(*graph));

    auto od_path = fs::temp_directory_path() /
        ("nomad_test_od_diamond_pretrip_" + std::to_string(enable_pretrip_reroute) + ".csv");
    {
        std::ofstream f(od_path);
        f << "origin_node,dest_node,count,mode,depart_mean_s,depart_std_s\n";
        // Early wave: 100 agents (ids 0..99), departures spread over roughly
        // [0,150s], sustaining pressure on the low-capacity edge A. The
        // discharge-gate-wait signal (gate_wait_s) resets to 0 on every
        // SUCCESSFUL exit (see ltm_model.cpp), so it only ever reflects time
        // since the LAST successful exit -- an intermittently-draining queue
        // would make gate_wait_s oscillate and possibly never exceed the
        // safe path's fixed 100s (path C+D) at the exact instant the late
        // agent's pretrip check happens to fire. To avoid that timing
        // sensitivity, capacity=5 veh/h -> capacity_veh_s ≈ 0.00139/s ->
        // refill to the 1.0 credit floor (discharge_burst_s=1.0) takes
        // ~720s once the initial free credit is spent by the very first
        // exit -- i.e. gate_wait_s grows MONOTONICALLY, uninterrupted, from
        // shortly after t=0 all the way past t=400 (the late agent's
        // departure), guaranteed to comfortably exceed 100s by then.
        f << "0,3,100,car,50,40\n";
        // Late agent (id 100): scheduled well after the wave has saturated
        // edge A, while it is still draining.
        f << "0,3,1,car,400,1\n";
    }
    sim.set_demand(std::make_shared<OdMatrixDemand>(
        OdMatrixDemand::from_csv(od_path)));

    constexpr AgentId kLateAgent = 100;

    sim.run();

    REQUIRE(sim.agent_hot().pretrip_rerouted[kLateAgent] == (enable_pretrip_reroute ? 1 : 0));
    REQUIRE(sim.agent_hot().reroute_count[kLateAgent] == 0);  // no post-departure confound

    auto route = sim.route_store().get_route(kLateAgent);
    REQUIRE_FALSE(route.empty());

    fs::remove(od_path);
    return route.front();
}

TEST_CASE("Simulation: pre-trip reroute diverts a later-departing agent around "
          "congestion caused by an earlier wave", "[integration]") {
    EdgeId control_first_edge   = run_diamond_and_capture_late_agent_first_edge(false);
    EdgeId treatment_first_edge = run_diamond_and_capture_late_agent_first_edge(true);

    REQUIRE(control_first_edge == 0);    // edge A: stale free-flow route, unchanged
    REQUIRE(treatment_first_edge == 2);  // edge C: diverted onto the safe path
}
