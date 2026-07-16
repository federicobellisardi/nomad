#include <nomad/config/scenario_config.hpp>
#include <nomad/core/simulation.hpp>
#include <nomad/demand/od_matrix.hpp>
#include <nomad/demand/synthetic.hpp>
#include <nomad/network/network_cleaner.hpp>
#include <nomad/network/osm_loader.hpp>
#include <nomad/output/geojson_writer.hpp>
#include <nomad/routing/astar_router.hpp>
#include <nomad/routing/ch_router.hpp>
#include <nomad/routing/route_cache.hpp>
#include <nomad/traffic/ltm_model.hpp>
#include <nomad/traffic/queue_model.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace nomad;

// ── CLI help ──────────────────────────────────────────────────────────────────
static void print_usage(const char* argv0) {
    std::cout << R"(
nomad — Network-based Open Mobility Agent Dynamics
Urban mobility simulator v0.1.0

Usage:
  )" << argv0 << R"( --config <scenario.json>   Run simulation from JSON config
  )" << argv0 << R"( --osm <file.pbf>           Quick run with default settings
  )" << argv0 << R"( --help                     Show this help

Example scenario.json:
  {
    "name": "my_city",
    "network": { "osm_pbf": "city.osm.pbf" },
    "simulation": { "start_time_s": 25200, "end_time_s": 32400 },
    "demand": { "source": "gravity", "synthetic": { "total_agents": 50000 } },
    "output": { "output_dir": "results", "writers": ["geojson"] }
  }

Supported demand sources:
  gravity           — Synthetic gravity model (no external data)
  radiation         — Synthetic radiation model
  od_csv            — OD matrix from CSV file
  activity_plan     — ActivitySim person_trips.csv

Traffic models:  queue (default), ltm
Routing:         CH (default), astar

See data/schemas/ for full JSON schema documentation.
)";
}

// ── Build simulation from config ──────────────────────────────────────────────
static int run_scenario(ScenarioConfig cfg) {
    auto t0 = std::chrono::steady_clock::now();

    // Validate
    std::string err = ScenarioConfigIO::validate(cfg);
    if (!err.empty()) {
        spdlog::error("Config validation failed: {}", err);
        return 1;
    }

    // ── Network loading ───────────────────────────────────────────────────────
    OsmLoader loader(OsmTagConfig::defaults()); // needed for turn_restrictions even if loading from cache
    std::shared_ptr<Graph> graph;

    if (cfg.network.graph_bin && std::filesystem::exists(*cfg.network.graph_bin)) {
        spdlog::info("Loading graph from cache: {}", cfg.network.graph_bin->string());
        graph = std::make_shared<Graph>(Graph::load(*cfg.network.graph_bin));
    } else {
        if (!cfg.network.osm_tag_config.empty())
            loader = OsmLoader(OsmTagConfig::load_yaml(cfg.network.osm_tag_config));
        spdlog::info("Loading OSM network: {}", cfg.network.osm_pbf.string());
        graph = std::make_shared<Graph>(
            loader.load_and_clean(cfg.network.osm_pbf, cfg.network.simplify_topology));
        if (cfg.network.graph_bin) {
            graph->save(*cfg.network.graph_bin);
            spdlog::info("Graph cache saved: {}", cfg.network.graph_bin->string());
        }
    }

    spdlog::info("Network: {} nodes, {} edges",
                  graph->num_nodes(), graph->num_edges());

    // ── Simulation setup ──────────────────────────────────────────────────────
    Simulation sim(cfg.simulation);
    sim.set_graph(graph);

    // ── Traffic model ─────────────────────────────────────────────────────────
    if (cfg.traffic.model == "ltm") {
        sim.set_traffic_model(std::make_unique<LtmTrafficModel>(*graph));
    } else {
        sim.set_traffic_model(std::make_unique<QueueTrafficModel>(*graph));
    }

    // ── Routing ───────────────────────────────────────────────────────────────
    auto cache = std::make_unique<RouteCache>(cfg.routing.cache_entries);
    if (cfg.routing.algorithm == "CH" || cfg.routing.algorithm == "ch") {
        CHRouter::PreprocessConfig pp;
        pp.num_threads = cfg.simulation.num_threads;
        pp.verbose     = true;
        auto ch = std::make_unique<CHRouter>(
            *graph, loader.turn_restrictions(), pp);

        if (cfg.routing.ch_cache && std::filesystem::exists(*cfg.routing.ch_cache)) {
            spdlog::info("Loading CH from cache: {}", cfg.routing.ch_cache->string());
            ch->load(*cfg.routing.ch_cache);
        } else if (cfg.routing.preprocess_ch) {
            spdlog::info("Preprocessing Contraction Hierarchies...");
            ch->preprocess();
            if (cfg.routing.ch_cache) {
                ch->save(*cfg.routing.ch_cache);
                spdlog::info("CH saved to {}", cfg.routing.ch_cache->string());
            }
        }
        ch->set_cache(cache.get());
        sim.set_router(std::move(ch));
    } else {
        auto astar = std::make_unique<AStarRouter>(*graph);
        astar->set_cache(cache.get());
        sim.set_router(std::move(astar));
    }

    // ── Mode distribution from config ─────────────────────────────────────────
    // Sample a mode from cfg.demand.modes weighted by cfg.demand.modes_split.
    auto sample_mode = [&](std::mt19937& rng) -> AgentMode {
        if (cfg.demand.modes.empty() || cfg.demand.modes.size() == 1) {
            const std::string& m = cfg.demand.modes.empty() ? "car" : cfg.demand.modes[0];
            if (m == "car")     return AgentMode::Car;
            if (m == "walk")    return AgentMode::Walk;
            if (m == "bike")    return AgentMode::Bike;
            if (m == "transit") return AgentMode::Transit;
            return AgentMode::Car;
        }
        // Build cumulative distribution
        float total = 0.0f;
        for (const auto& m : cfg.demand.modes) {
            auto it = cfg.demand.modes_split.find(m);
            total += (it != cfg.demand.modes_split.end()) ? it->second : 1.0f;
        }
        float r = std::uniform_real_distribution<float>(0.0f, total)(rng);
        float acc = 0.0f;
        for (const auto& m : cfg.demand.modes) {
            auto it = cfg.demand.modes_split.find(m);
            acc += (it != cfg.demand.modes_split.end()) ? it->second : 1.0f;
            if (r <= acc) {
                if (m == "car")     return AgentMode::Car;
                if (m == "walk")    return AgentMode::Walk;
                if (m == "bike")    return AgentMode::Bike;
                if (m == "transit") return AgentMode::Transit;
            }
        }
        return AgentMode::Car;
    };
    (void)sample_mode; // used by demand models when multimodal is implemented in Phase 4

    // Log selected modes
    std::string modes_str;
    for (const auto& m : cfg.demand.modes) {
        if (!modes_str.empty()) modes_str += ", ";
        modes_str += m;
        auto it = cfg.demand.modes_split.find(m);
        if (it != cfg.demand.modes_split.end())
            modes_str += "(" + std::to_string(static_cast<int>(it->second * 100)) + "%)";
    }
    spdlog::info("Transport modes: [{}]", modes_str);

    // ── Demand ────────────────────────────────────────────────────────────────
    // Primary mode for synthetic demand (first in modes list)
    AgentMode primary_mode = AgentMode::Car;
    if (!cfg.demand.modes.empty()) {
        const auto& m0 = cfg.demand.modes[0];
        if (m0 == "walk")    primary_mode = AgentMode::Walk;
        else if (m0 == "bike")    primary_mode = AgentMode::Bike;
        else if (m0 == "transit") primary_mode = AgentMode::Transit;
    }

    if (cfg.demand.source == "od_csv") {
        // Build allowed-modes list from config (empty = load all modes)
        std::vector<AgentMode> allowed_modes;
        for (const auto& m : cfg.demand.modes) {
            if (m == "car")     allowed_modes.push_back(AgentMode::Car);
            else if (m == "walk")    allowed_modes.push_back(AgentMode::Walk);
            else if (m == "bike")    allowed_modes.push_back(AgentMode::Bike);
            else if (m == "transit") allowed_modes.push_back(AgentMode::Transit);
        }
        sim.set_demand(std::make_shared<OdMatrixDemand>(
            OdMatrixDemand::from_csv(*cfg.demand.od_csv, 42,
                cfg.simulation.start_time, cfg.simulation.end_time,
                allowed_modes, cfg.demand.demand_scale)));
    } else if (cfg.demand.source == "gravity") {
        GravityDemand::Config gc;
        gc.total_agents = cfg.demand.synthetic.total_agents;
        gc.beta         = cfg.demand.synthetic.beta;
        gc.peak_mean_s  = cfg.demand.synthetic.peak_mean_s;
        gc.peak_std_s   = cfg.demand.synthetic.peak_std_s;
        gc.seed         = cfg.demand.synthetic.seed;
        gc.mode         = primary_mode;
        sim.set_demand(std::make_shared<GravityDemand>(gc));
    } else if (cfg.demand.source == "radiation") {
        RadiationDemand::Config rc;
        rc.total_agents = cfg.demand.synthetic.total_agents;
        rc.seed         = cfg.demand.synthetic.seed;
        rc.mode         = primary_mode;
        sim.set_demand(std::make_shared<RadiationDemand>(rc));
    } else {
        spdlog::error("Unknown demand source: {}", cfg.demand.source);
        return 1;
    }

    // ── Timestamped output subdir ─────────────────────────────────────────────
    // Appends "{date}_{HH-MM}_{HH-MM}" to output_dir, e.g.
    //   results/palma_de_mallorca/2022-02-08_06-00_11-00
    {
        auto fmt_hm = [](SimTime t) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%02d-%02d",
                static_cast<int>(t) / 3600,
                (static_cast<int>(t) % 3600) / 60);
            return std::string(buf);
        };
        std::string subdir;
        if (!cfg.sim_date.empty()) subdir = cfg.sim_date + "_";
        subdir += fmt_hm(cfg.simulation.start_time) + "_"
               +  fmt_hm(cfg.simulation.end_time);
        cfg.output.output_dir /= subdir;
        spdlog::info("Output dir: {}", cfg.output.output_dir.string());
    }

    // ── Output writers ────────────────────────────────────────────────────────
    std::filesystem::create_directories(cfg.output.output_dir);
    for (const auto& writer_name : cfg.output.writers) {
        if (writer_name == "geojson") {
            sim.add_output_writer(std::make_unique<GeoJsonWriter>(
                cfg.output.output_dir,
                static_cast<float>(cfg.output.snapshot_interval_s)));
        }
    }

    // ── Run ───────────────────────────────────────────────────────────────────
    spdlog::info("Starting simulation: {:.0f}s – {:.0f}s",
                  cfg.simulation.start_time, cfg.simulation.end_time);
    sim.run();

    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    spdlog::info("Simulation complete in {:.1f}s. Events processed: {}",
                  elapsed, sim.events_processed());
    return 0;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    // Configure spdlog
    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] [%l] %v");

    if (argc < 2) { print_usage(argv[0]); return 1; }

    std::string arg1(argv[1]);

    if (arg1 == "--help" || arg1 == "-h") {
        print_usage(argv[0]);
        return 0;
    }

    ScenarioConfig cfg;

    if (arg1 == "--config" && argc >= 3) {
        try {
            cfg = ScenarioConfigIO::load(argv[2]);
        } catch (const std::exception& e) {
            spdlog::error("Failed to load config {}: {}", argv[2], e.what());
            return 1;
        }
    } else if (arg1 == "--osm" && argc >= 3) {
        // Quick run: OSM file, gravity demand, GeoJSON output
        cfg.network.osm_pbf          = argv[2];
        cfg.demand.source            = "gravity";
        cfg.demand.synthetic.total_agents = 10000;
        cfg.output.output_dir        = "nomad_output";
        cfg.output.writers           = {"geojson"};
        spdlog::info("Quick run mode: OSM={}, 10k agents, GeoJSON output", argv[2]);
    } else {
        print_usage(argv[0]);
        return 1;
    }

    return run_scenario(cfg);
}
