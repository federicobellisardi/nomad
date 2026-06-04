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
#include <nomad/traffic/queue_model.hpp>
#include <nomad/traffic/ltm_model.hpp>
#include <nomad/transit/gtfs_loader.hpp>

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
        // Zero-copy edge lengths
        .def("edge_lengths", [](const std::shared_ptr<Graph>& g) {
            auto owner = py::cast(g);
            return py::array_t<float>(
                {static_cast<py::ssize_t>(g->num_edges())},
                {static_cast<py::ssize_t>(sizeof(EdgeData))},
                &g->edges[0].length_m, owner
            );
        }, py::keep_alive<0,1>())
        .def("validate", &Graph::validate);

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

    // ── OdMatrixDemand ────────────────────────────────────────────────────────
    py::class_<OdMatrixDemand, IDemandModel, std::shared_ptr<OdMatrixDemand>>(m, "OdMatrixDemand")
        .def_static("from_csv", [](const std::string& path, uint32_t seed) {
            return std::make_shared<OdMatrixDemand>(
                OdMatrixDemand::from_csv(path, seed));
        }, py::arg("csv_path"), py::arg("seed") = 42);

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

    // ── Simulation ────────────────────────────────────────────────────────────
    py::class_<Simulation>(m, "Simulation")
        .def(py::init<SimulationConfig>(), py::arg("config") = SimulationConfig{})
        .def("set_graph",   [](Simulation& s, std::shared_ptr<Graph> g) { s.set_graph(g); })
        .def("set_demand",  [](Simulation& s, std::shared_ptr<IDemandModel> d) { s.set_demand(d); })
        .def("run",         &Simulation::run,
             py::call_guard<py::gil_scoped_release>())
        .def("run_until",   &Simulation::run_until,
             py::arg("time"), py::call_guard<py::gil_scoped_release>())
        .def("step",        &Simulation::step,
             py::call_guard<py::gil_scoped_release>())
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
