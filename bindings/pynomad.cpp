#include <nomad/config/scenario_config.hpp>
#include <nomad/core/agent.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/core/types.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/demand/synthetic.hpp>
#include <nomad/demand/mode_choice.hpp>
#include <nomad/network/multilayer_graph.hpp>
#include <nomad/network/osm_loader.hpp>
#include <nomad/network/network_cleaner.hpp>
#include <nomad/output/geojson_writer.hpp>
#include <nomad/output/parquet_writer.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/routing/ch_router.hpp>
#include <nomad/routing/route_cache.hpp>
#include <nomad/routing/router.hpp>
#include <nomad/traffic/queue_model.hpp>
#include <nomad/traffic/ltm_model.hpp>
#include <nomad/transit/gtfs_loader.hpp>

#include <algorithm>
#include <tuple>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace nomad;

// ── Zero-copy NumPy helper ────────────────────────────────────────────────────
template <typename T>
py::array_t<T> make_array(const std::vector<T>& vec, py::object owner) {
    return py::array_t<T>(
        {static_cast<py::ssize_t>(vec.size())},
        {static_cast<py::ssize_t>(sizeof(T))},
        vec.data(),
        owner
    );
}

// ── Event type string parsing ─────────────────────────────────────────────────
static EventType parse_event_type(const std::string& s) {
    if (s == "AgentDepart")          return EventType::AgentDepart;
    if (s == "AgentEnterLink")       return EventType::AgentEnterLink;
    if (s == "AgentExitLink")        return EventType::AgentExitLink;
    if (s == "AgentArriveActivity")  return EventType::AgentArriveActivity;
    if (s == "AgentReroute")         return EventType::AgentReroute;
    if (s == "SignalPhaseChange")    return EventType::SignalPhaseChange;
    if (s == "SnapshotDump")         return EventType::SnapshotDump;
    throw py::value_error("Unknown event type: " + s);
}

// Simulation::run()/run_until()/step() silently no-op instead of erroring
// when no router (and/or no traffic model) has been attached — pre-routing
// short-circuits on a null router_ (see simulation.cpp: compute_route /
// generate_initial_routes), so agents are generated but never routed or
// moved, and the run "succeeds" with an empty result. This turns that into
// an explicit, actionable Python exception at the point of use.
static void require_ready(const Simulation& s) {
    if (!s.has_router())
        throw py::value_error(
            "Simulation has no router attached — call set_router(AStarRouter(graph)) "
            "(mode-aware; required for walk/bike) or set_router(CHRouter(graph)) "
            "(car-only) before run()/run_until()/step(). Without a router, "
            "pre-routing silently no-ops and no agent ever moves.");
    if (!s.has_traffic_model())
        throw py::value_error(
            "Simulation has no traffic model attached — call "
            "set_traffic_model(QueueTrafficModel(graph)) or "
            "set_traffic_model(LtmTrafficModel(graph)) before "
            "run()/run_until()/step().");
}

PYBIND11_MODULE(_nomad_core, m) {
    m.doc() = "nomad urban mobility simulator — C++ core";

    // ── Types / enums ─────────────────────────────────────────────────────────
    py::enum_<AgentMode>(m, "AgentMode")
        .value("Car",     AgentMode::Car)
        .value("Transit", AgentMode::Transit)
        .value("Bike",    AgentMode::Bike)
        .value("Walk",    AgentMode::Walk)
        .value("Idle",    AgentMode::Idle)
        .export_values();

    py::enum_<AgentState>(m, "AgentState")
        .value("Waiting",    AgentState::Waiting)
        .value("OnLink",     AgentState::OnLink)
        .value("AtActivity", AgentState::AtActivity)
        .value("Arrived",    AgentState::Arrived)
        .export_values();

    // Bound only so build_test_graph() callers (and tests) don't have to
    // hardcode RoadClass's raw uint8 values — not otherwise required by the
    // router/traffic-model binding work below.
    py::enum_<RoadClass>(m, "RoadClass")
        .value("Motorway",      RoadClass::Motorway)
        .value("MotorwayLink",  RoadClass::MotorwayLink)
        .value("Trunk",         RoadClass::Trunk)
        .value("TrunkLink",     RoadClass::TrunkLink)
        .value("Primary",       RoadClass::Primary)
        .value("PrimaryLink",   RoadClass::PrimaryLink)
        .value("Secondary",     RoadClass::Secondary)
        .value("SecondaryLink", RoadClass::SecondaryLink)
        .value("Tertiary",      RoadClass::Tertiary)
        .value("TertiaryLink",  RoadClass::TertiaryLink)
        .value("Residential",   RoadClass::Residential)
        .value("LivingStreet",  RoadClass::LivingStreet)
        .value("Service",       RoadClass::Service)
        .value("Unclassified",  RoadClass::Unclassified)
        .value("Track",         RoadClass::Track)
        .value("Cycleway",      RoadClass::Cycleway)
        .value("Footway",       RoadClass::Footway)
        .value("Path",          RoadClass::Path)
        .value("Steps",         RoadClass::Steps)
        .value("Unknown",       RoadClass::Unknown)
        .export_values();

    // ── Graph ─────────────────────────────────────────────────────────────────
    py::class_<Graph, std::shared_ptr<Graph>>(m, "Graph")
        .def_property_readonly("num_nodes", &Graph::num_nodes)
        .def_property_readonly("num_edges", &Graph::num_edges)
        // Zero-copy node coordinate array: shape (N, 2) = [lon, lat]
        .def("node_coords", [](const std::shared_ptr<Graph>& g) -> py::array_t<float> {
            auto owner = py::cast(g);
            std::size_t N = g->num_nodes();
            return py::array_t<float>(
                {static_cast<py::ssize_t>(N), static_cast<py::ssize_t>(2)},
                {static_cast<py::ssize_t>(sizeof(NodeData)),
                 static_cast<py::ssize_t>(sizeof(float))},
                &g->nodes[0].lon,
                owner
            );
        }, py::keep_alive<0,1>())
        // Zero-copy edge lengths [m]
        .def("edge_lengths", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<float>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].length_m, owner
            );
        }, py::keep_alive<0,1>())
        // Zero-copy edge target node IDs (parallel to edges)
        .def("edge_targets", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<uint32_t>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                reinterpret_cast<const uint32_t*>(&g->edges[0].target), owner
            );
        }, py::keep_alive<0,1>())
        // CSR row pointers — size (num_nodes + 1). diff gives out-degree per node.
        .def("row_ptr", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<uint32_t>(
                {static_cast<py::ssize_t>(g->row_ptr.size())},
                {static_cast<py::ssize_t>(sizeof(uint32_t))},
                g->row_ptr.data(), owner
            );
        }, py::keep_alive<0,1>())
        // Free-flow speeds [m/s]
        .def("edge_speeds", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<float>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].free_flow_speed, owner
            );
        }, py::keep_alive<0,1>())
        // Road class (uint8, RoadClass enum)
        .def("edge_road_class", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<uint8_t>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].road_class, owner
            );
        }, py::keep_alive<0,1>())
        // Capacity [veh/h]
        .def("edge_capacity", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<float>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].capacity, owner
            );
        }, py::keep_alive<0,1>())
        // Index into way_names/way_ids for each edge
        .def("edge_way_meta_idx", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<uint16_t>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].way_meta_idx, owner
            );
        }, py::keep_alive<0,1>())
        // Intersection type (uint8, IntersectionType enum), per node
        .def("node_intersection_type", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<uint8_t>(
                {static_cast<py::ssize_t>(g->num_nodes())},
                {static_cast<py::ssize_t>(sizeof(NodeData))},
                &g->nodes[0].intersection_type, owner
            );
        }, py::keep_alive<0,1>())
        // OSM way name for a given way_meta_idx (from edge_way_meta_idx())
        .def("way_name", [](const std::shared_ptr<Graph>& g, uint16_t idx) {
            return idx < g->way_names.size() ? g->way_names[idx] : std::string{};
        }, py::arg("way_meta_idx"))
        .def("validate", &Graph::validate)
        .def("save", [](const std::shared_ptr<Graph>& g, const std::string& path) {
            g->save(path);
        }, py::arg("path"), "Save graph to binary cache file.")
        .def_static("load", [](const std::string& path) {
            return std::make_shared<Graph>(Graph::load(path));
        }, py::arg("path"), "Load graph from binary cache file.")
        .def("mode_free_flow_time", &Graph::mode_free_flow_time,
             py::arg("edge_id"), py::arg("mode"), py::arg("walk_speed_ms") = 1.39f,
             py::arg("bike_speed_ms") = 4.17f,
             "Deterministic per-edge travel time [s] for a mode (car/transit: "
             "edge's own free-flow speed; walk/bike: capped at the mode's own "
             "pace) — walk/bike never experience congestion (no ped/bike "
             "traffic model), so this IS their real travel time, not an "
             "estimate. See include/nomad/core/graph.hpp for the exact formula.");

    // ── OsmLoader ─────────────────────────────────────────────────────────────
    py::class_<OsmLoader>(m, "OsmLoader")
        .def(py::init<>())
        .def("load", [](OsmLoader& self, const std::string& path) {
            return std::make_shared<Graph>(self.load(path));
        }, py::arg("osm_pbf_path"))
        .def("load_and_clean", [](OsmLoader& self, const std::string& path,
                                    bool simplify) {
            return std::make_shared<Graph>(self.load_and_clean(path, simplify));
        }, py::arg("osm_pbf_path"), py::arg("simplify_topology") = true);

    // ── NetworkCleaner ────────────────────────────────────────────────────────
    py::class_<NetworkCleaner>(m, "NetworkCleaner")
        .def(py::init<>())
        .def("clean", [](NetworkCleaner& c, std::shared_ptr<Graph> g) {
            return std::make_shared<Graph>(c.clean(std::move(*g)));
        });

    // ── SimulationConfig ──────────────────────────────────────────────────────
    py::class_<SimulationConfig>(m, "SimulationConfig")
        .def(py::init<>())
        .def_readwrite("start_time",          &SimulationConfig::start_time)
        .def_readwrite("end_time",            &SimulationConfig::end_time)
        .def_readwrite("sync_window_s",       &SimulationConfig::sync_window_s)
        .def_readwrite("reroute_thresh",      &SimulationConfig::reroute_thresh)
        .def_readwrite("num_threads",         &SimulationConfig::num_threads)
        .def_readwrite("store_traces",        &SimulationConfig::store_traces)
        .def_readwrite("traffic_model",       &SimulationConfig::traffic_model)
        .def_readwrite("router",              &SimulationConfig::router)
        .def_readwrite("snapshot_interval_s", &SimulationConfig::snapshot_interval_s);

    // ── ScenarioConfig ────────────────────────────────────────────────────────
    py::class_<ScenarioConfig>(m, "ScenarioConfig")
        .def(py::init<>())
        .def_readwrite("name",       &ScenarioConfig::name)
        .def_readwrite("simulation", &ScenarioConfig::simulation)
        .def_static("load", [](const std::string& path) {
            return ScenarioConfigIO::load(path);
        });

    // ── IDemandModel (base, must be registered before derived classes) ───────
    py::class_<IDemandModel, std::shared_ptr<IDemandModel>>(m, "IDemandModel");

    // ── OdMatrixDemand ────────────────────────────────────────────────────────
    py::class_<OdMatrixDemand, IDemandModel, std::shared_ptr<OdMatrixDemand>>(m, "OdMatrixDemand")
        .def_static("from_csv", [](const std::string& path, uint32_t seed,
                                    float time_min_s, float time_max_s,
                                    const std::vector<std::string>& modes) {
            std::vector<AgentMode> allowed;
            for (const auto& m : modes) {
                if      (m == "car")     allowed.push_back(AgentMode::Car);
                else if (m == "walk")    allowed.push_back(AgentMode::Walk);
                else if (m == "bike")    allowed.push_back(AgentMode::Bike);
                else if (m == "transit") allowed.push_back(AgentMode::Transit);
            }
            return std::make_shared<OdMatrixDemand>(
                OdMatrixDemand::from_csv(path, seed, time_min_s, time_max_s, allowed));
        }, py::arg("csv_path"), py::arg("seed") = 42,
           py::arg("time_min_s") = -1e9f, py::arg("time_max_s") = 1e9f,
           py::arg("modes") = std::vector<std::string>{});

    // ── GravityDemand ─────────────────────────────────────────────────────────
    py::class_<GravityDemand, IDemandModel, std::shared_ptr<GravityDemand>>(m, "GravityDemand")
        .def(py::init([](uint32_t total_agents, float beta,
                          float peak_mean_s, uint32_t seed) {
            GravityDemand::Config cfg;
            cfg.total_agents = total_agents;
            cfg.beta         = beta;
            cfg.peak_mean_s  = peak_mean_s;
            cfg.seed         = seed;
            return std::make_shared<GravityDemand>(cfg);
        }), py::arg("total_agents") = 10000,
            py::arg("beta")         = 0.003f,
            py::arg("peak_mean_s")  = 28800.0f,
            py::arg("seed")         = 42);

    // ── Routing ───────────────────────────────────────────────────────────────
    // RoutingRequest/Route: plain data structs (router.hpp). Bound so a route
    // (and its total time) can be computed directly from Python WITHOUT
    // running a full Simulation — the only viable way to get walk/bike
    // per-edge timing cheaply, since those modes never reroute (no
    // congestion feedback) and their pre-computed route IS their real path
    // for the whole trip (see RouteStore in simulation.hpp: routes_ is
    // populated once at generation and never cleared per-agent).
    py::class_<RoutingRequest>(m, "RoutingRequest")
        .def(py::init([](NodeId origin, NodeId destination, SimTime departure_time, AgentMode mode) {
            return RoutingRequest{origin, destination, departure_time, mode};
        }), py::arg("origin"), py::arg("destination"), py::arg("departure_time") = 0.0,
            py::arg("mode") = AgentMode::Car)
        .def_readwrite("origin", &RoutingRequest::origin)
        .def_readwrite("destination", &RoutingRequest::destination)
        .def_readwrite("departure_time", &RoutingRequest::departure_time)
        .def_readwrite("mode", &RoutingRequest::mode);

    py::class_<Route>(m, "Route")
        .def_readonly("edges", &Route::edges)
        .def_readonly("estimated_time_s", &Route::estimated_time_s)
        .def_readonly("estimated_dist_m", &Route::estimated_dist_m)
        .def_readonly("is_valid", &Route::is_valid);

    // Base class registered first (required for the unique_ptr<IRouter>
    // up-cast used by Simulation::set_router below). No constructor: IRouter
    // is abstract.
    //
    // py::smart_holder (not the default std::unique_ptr holder): pybind11 3.x
    // requires it for any class passed as `std::unique_ptr<Base>` from Python
    // into a C++ function that takes ownership (Simulation::set_router/
    // set_traffic_model) — without it, pybind11 raises at call time:
    // "Passing std::unique_ptr<T> from Python to C++ requires
    // py::class_<T, py::smart_holder>". Confirmed by trial: switching these
    // six classes to py::smart_holder is what made tests/python/
    // test_multimodal_bindings.py pass.
    py::class_<IRouter, py::smart_holder>(m, "IRouter")
        .def("route", &IRouter::route, py::arg("request"),
             "Compute a single route (blocking, single-threaded query) — "
             "works on both AStarRouter and CHRouter (CH ignores mode).");

    py::class_<AStarRouter, IRouter, py::smart_holder>(m, "AStarRouter")
        .def(py::init([](std::shared_ptr<Graph> g) {
            return std::make_unique<AStarRouter>(*g);
        }), py::arg("graph"), py::keep_alive<1, 2>(),
            "Mode-aware router (car/walk/bike): filters edges per mode via "
            "road_class_accessible and caps effective speed at the mode's own "
            "pace. Required for any scenario using non-car modes — CHRouter "
            "ignores AgentMode at query time (see CHRouter docs).")
        .def_property_readonly("name", &AStarRouter::router_name);

    py::class_<CHRouter, IRouter, py::smart_holder>(m, "CHRouter")
        .def(py::init([](std::shared_ptr<Graph> g) {
            return std::make_unique<CHRouter>(*g);
        }), py::arg("graph"), py::keep_alive<1, 2>(),
            "Contraction Hierarchies router: ~1000x faster car-only queries. "
            "Ignores AgentMode at query time (routes are always car-optimal) "
            "— do not use for scenarios with walk/bike agents. Call "
            "preprocess() once before routing.")
        .def("preprocess", &CHRouter::preprocess,
             "Offline contraction step; required once before this router can "
             "answer queries (unless loaded from a cache via load()).")
        .def_property_readonly("is_preprocessed", &CHRouter::is_preprocessed)
        .def_property_readonly("name", &CHRouter::router_name);

    // ── Traffic models ────────────────────────────────────────────────────────
    py::class_<ITrafficModel, py::smart_holder>(m, "ITrafficModel");

    py::class_<QueueTrafficModel, ITrafficModel, py::smart_holder>(m, "QueueTrafficModel")
        .def(py::init([](std::shared_ptr<Graph> g) {
            return std::make_unique<QueueTrafficModel>(*g);
        }), py::arg("graph"), py::keep_alive<1, 2>(),
            "BPR volume-delay queue model (default, fastest). Only "
            "AgentMode.Car interacts with it — walk/bike agents never touch "
            "link occupancy/inflow/outflow (see Simulation.link_states docs).")
        .def_property_readonly("name", &QueueTrafficModel::model_name);

    py::class_<LtmTrafficModel, ITrafficModel, py::smart_holder>(m, "LtmTrafficModel")
        .def(py::init([](std::shared_ptr<Graph> g) {
            return std::make_unique<LtmTrafficModel>(*g);
        }), py::arg("graph"), py::keep_alive<1, 2>(),
            "Single-cell CTM-lite traffic model with real spillback. Same "
            "car-only interaction caveat as QueueTrafficModel.")
        .def_property_readonly("name", &LtmTrafficModel::model_name);

    // ── Simulation ────────────────────────────────────────────────────────────
    py::class_<Simulation>(m, "Simulation")
        .def(py::init<SimulationConfig>(), py::arg("config") = SimulationConfig{})
        .def("set_graph",   [](Simulation& s, std::shared_ptr<Graph> g) { s.set_graph(g); })
        .def("set_demand",  [](Simulation& s, std::shared_ptr<IDemandModel> d) { s.set_demand(d); })
        .def("set_router",         &Simulation::set_router, py::arg("router"),
             "Attach an AStarRouter or CHRouter. Mandatory before run()/"
             "run_until()/step() — see require_ready in this file.")
        .def("set_traffic_model",  &Simulation::set_traffic_model, py::arg("traffic_model"),
             "Attach a QueueTrafficModel or LtmTrafficModel. Mandatory before "
             "run()/run_until()/step().")
        .def_property_readonly("has_router",        &Simulation::has_router)
        .def_property_readonly("has_traffic_model", &Simulation::has_traffic_model)
        .def("run", [](Simulation& s) {
            require_ready(s);
            py::gil_scoped_release release;
            s.run();
        })
        .def("run_until", [](Simulation& s, SimTime t) {
            require_ready(s);
            py::gil_scoped_release release;
            s.run_until(t);
        }, py::arg("time"))
        .def("step", [](Simulation& s) {
            require_ready(s);
            py::gil_scoped_release release;
            s.step();
        })
        .def("current_time",   &Simulation::current_time)
        .def("active_agents",  &Simulation::active_agents)
        .def("events_processed", &Simulation::events_processed)
        // Zero-copy state arrays
        .def("agent_edges", [](const Simulation& s) {
            return make_array(s.agent_hot().current_edge, py::cast(&s));
        }, py::keep_alive<0,1>())
        .def("agent_states", [](const Simulation& s) -> py::array_t<uint8_t> {
            const auto& hot = s.agent_hot();
            return py::array_t<uint8_t>(
                {static_cast<py::ssize_t>(hot.state.size())},
                {static_cast<py::ssize_t>(sizeof(AgentState))},
                reinterpret_cast<const uint8_t*>(hot.state.data()),
                py::cast(&s)
            );
        }, py::keep_alive<0,1>())
        // Link state array: shape (num_edges, 4) = [occupancy, inflow, outflow, travel_time]
        .def("link_states", [](const Simulation& s) -> py::array_t<float> {
            const auto states = s.traffic().link_states();
            std::size_t E = states.size();
            py::array_t<float> arr({static_cast<py::ssize_t>(E),
                                     static_cast<py::ssize_t>(4)});
            auto buf = arr.mutable_unchecked<2>();
            for (std::size_t i = 0; i < E; ++i) {
                buf(i, 0) = states[i].occupancy    .load();
                buf(i, 1) = states[i].inflow_rate  .load();
                buf(i, 2) = states[i].outflow_rate .load();
                buf(i, 3) = states[i].travel_time_s.load();
            }
            return arr;
        })
        // Python callback hooks (GIL acquired on callback entry)
        .def("register_hook", [](Simulation& s, const std::string& event_type_str,
                                  py::function callback) {
            EventType etype = parse_event_type(event_type_str);
            s.register_hook(etype, [callback](const Event& e) {
                py::gil_scoped_acquire acq;
                callback(e.time, static_cast<uint32_t>(e.agent), e.payload);
            });
        }, py::arg("event_type"), py::arg("callback"))
        .def("add_geojson_writer", [](Simulation& s, const std::string& output_dir,
                                       float interval) {
            s.add_output_writer(
                std::make_unique<GeoJsonWriter>(output_dir, interval));
        }, py::arg("output_dir"), py::arg("snapshot_interval_s") = 300.0f);

    // ── Test-only synthetic Graph builder ─────────────────────────────────────
    // Graph has no other Python-visible constructor (OsmLoader.load_and_clean
    // or Graph.load(cache) are the only ways to get one) — this exists purely
    // so small artificial networks can be unit-tested from Python without a
    // real .osm.pbf file. Real cities must still go through OsmLoader.
    m.def("build_test_graph", [](
              const std::vector<std::tuple<uint32_t, uint32_t, float, float, float, uint8_t>>& directed_edges,
              uint32_t num_nodes) {
        // Each tuple: (from_node, to_node, length_m, free_flow_speed_ms,
        // capacity_veh_h, road_class). Order is arbitrary — sorted by
        // from_node here to build the CSR (Graph::row_ptr/col_idx).
        std::vector<size_t> order(directed_edges.size());
        for (size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return std::get<0>(directed_edges[a]) < std::get<0>(directed_edges[b]);
        });

        Graph g;
        g.nodes.resize(num_nodes);
        for (uint32_t i = 0; i < num_nodes; ++i) {
            // Placeholder coordinates spaced ~111m apart (0.001 deg longitude)
            // — only used for A*'s haversine heuristic; keep test edge
            // lengths on a comparable scale (hundreds of meters) so the
            // heuristic stays admissible (haversine <= true route cost).
            g.nodes[i].lon = static_cast<float>(i) * 0.001f;
            g.nodes[i].lat = 0.0f;
        }
        g.row_ptr.assign(num_nodes + 1, 0);
        for (auto idx : order) {
            uint32_t from = std::get<0>(directed_edges[idx]);
            if (from >= num_nodes)
                throw py::value_error("edge from_node out of range for num_nodes");
            ++g.row_ptr[from + 1];
        }
        for (uint32_t i = 0; i < num_nodes; ++i) g.row_ptr[i + 1] += g.row_ptr[i];

        g.edges.resize(directed_edges.size());
        g.col_idx.resize(directed_edges.size());
        for (size_t k = 0; k < order.size(); ++k) {
            const auto& [from, to, len, speed, cap, rc] = directed_edges[order[k]];
            (void)from;
            if (to >= num_nodes)
                throw py::value_error("edge to_node out of range for num_nodes");
            EdgeData ed{};
            ed.target          = to;
            ed.length_m        = len;
            ed.free_flow_speed = speed;
            ed.capacity        = cap;
            ed.road_class      = rc;
            g.edges[k]   = ed;
            g.col_idx[k] = static_cast<EdgeId>(k);
        }
        g.geom_ptr.assign(g.edges.size() + 1, 0);
        return std::make_shared<Graph>(std::move(g));
    }, py::arg("edges"), py::arg("num_nodes"),
    "Build a tiny in-memory Graph for tests from an explicit directed-edge "
    "list of (from_node, to_node, length_m, free_flow_speed_ms, "
    "capacity_veh_h, road_class) tuples. Test/synthetic-network use only.");

    // ── Route cache diagnostics ───────────────────────────────────────────────
    py::class_<RouteCache>(m, "RouteCache")
        .def(py::init<std::size_t>(), py::arg("max_entries") = 500000)
        .def("size", &RouteCache::size)
        .def("clear", &RouteCache::clear)
        .def("stats", [](const RouteCache& c) {
            auto s = c.stats();
            py::dict d;
            d["hits"]         = s.hits;
            d["misses"]       = s.misses;
            d["evictions"]    = s.evictions;
            d["invalidations"]= s.invalidations;
            return d;
        });

    // ── Version ───────────────────────────────────────────────────────────────
    m.attr("__version__") = "0.1.0";
    m.attr("__author__")  = "nomad contributors";
}
