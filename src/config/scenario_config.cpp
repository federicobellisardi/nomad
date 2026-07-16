#include <nomad/config/scenario_config.hpp>

#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace nomad {

using json = nlohmann::json;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Accept either a plain number (seconds since midnight) or a datetime string
// in "YYYY-MM-DD HH:MM:SS" format.  Only the time portion is used.
static SimTime parse_sim_time(const json& val) {
    if (val.is_number()) return val.get<SimTime>();
    if (val.is_string()) {
        const std::string& s = val.get<std::string>();
        auto sp = s.find(' ');
        const char* t = (sp != std::string::npos) ? s.c_str() + sp + 1 : s.c_str();
        int h = 0, m = 0, sec = 0;
        std::sscanf(t, "%d:%d:%d", &h, &m, &sec);
        return static_cast<SimTime>(h * 3600 + m * 60 + sec);
    }
    throw std::runtime_error(
        "start_time / end_time must be a number (seconds) or \"YYYY-MM-DD HH:MM:SS\"");
}


// ── Load ──────────────────────────────────────────────────────────────────────
ScenarioConfig ScenarioConfigIO::load(const std::filesystem::path& json_path) {
    std::ifstream f(json_path);
    if (!f) throw std::runtime_error("Cannot open config: " + json_path.string());
    json j;
    f >> j;

    ScenarioConfig cfg;
    cfg.name    = j.value("name", cfg.name);
    cfg.version = j.value("version", cfg.version);

    // ── simulation ──────────────────────────────────────────────────────────
    if (j.contains("simulation")) {
        const auto& s = j["simulation"];
        // Accept "start_time" (datetime string, new format) or
        // legacy "start_time_s" (plain seconds).
        if (s.contains("start_time")) {
            cfg.simulation.start_time = parse_sim_time(s["start_time"]);
            if (s["start_time"].is_string()) {
                const std::string& raw = s["start_time"].get<std::string>();
                auto sp = raw.find(' ');
                if (sp != std::string::npos) cfg.sim_date = raw.substr(0, sp);
            }
        } else if (s.contains("start_time_s"))
            cfg.simulation.start_time = parse_sim_time(s["start_time_s"]);

        if (s.contains("end_time"))
            cfg.simulation.end_time = parse_sim_time(s["end_time"]);
        else if (s.contains("end_time_s"))
            cfg.simulation.end_time = parse_sim_time(s["end_time_s"]);
        cfg.simulation.sync_window_s         = s.value("sync_window_s",         cfg.simulation.sync_window_s);
        cfg.simulation.reroute_thresh        = s.value("reroute_thresh",        cfg.simulation.reroute_thresh);
        cfg.simulation.reroute_interval_s    = s.value("reroute_interval_s",    cfg.simulation.reroute_interval_s);
        cfg.simulation.stuck_threshold_ratio = s.value("stuck_threshold_ratio", cfg.simulation.stuck_threshold_ratio);
        cfg.simulation.teleport_interval_s   = s.value("teleport_interval_s",   cfg.simulation.teleport_interval_s);
        cfg.simulation.max_reroutes          = s.value("max_reroutes",          cfg.simulation.max_reroutes);
        cfg.simulation.stuck_max_hours       = s.value("stuck_max_hours",       cfg.simulation.stuck_max_hours);
        cfg.simulation.num_threads           = s.value("num_threads",           cfg.simulation.num_threads);
        cfg.simulation.store_traces  = s.value("store_traces", cfg.simulation.store_traces);
        cfg.simulation.traffic_model = s.value("traffic_model",cfg.simulation.traffic_model);
        cfg.simulation.router        = s.value("router",       cfg.simulation.router);
        cfg.simulation.snapshot_interval_s =
            s.value("snapshot_interval_s", cfg.simulation.snapshot_interval_s);
    }

    // ── network ─────────────────────────────────────────────────────────────
    if (j.contains("network")) {
        const auto& n = j["network"];
        cfg.network.osm_pbf            = n.value("osm_pbf", std::string{});
        cfg.network.simplify_topology  = n.value("simplify_topology", true);
        cfg.network.extract_transit    = n.value("extract_transit",   true);
        if (n.contains("osm_tag_config"))
            cfg.network.osm_tag_config = n["osm_tag_config"].get<std::string>();
        if (n.contains("gtfs_dir"))
            cfg.network.gtfs_dir = n["gtfs_dir"].get<std::string>();
        if (n.contains("graph_bin"))
            cfg.network.graph_bin = n["graph_bin"].get<std::string>();
        if (n.contains("bbox")) {
            const auto& b = n["bbox"];
            cfg.network.bbox = NetworkConfig::BBox{
                b.value("min_lon", -180.0f), b.value("min_lat", -90.0f),
                b.value("max_lon",  180.0f), b.value("max_lat",  90.0f)};
        }
    }

    // ── routing ─────────────────────────────────────────────────────────────
    if (j.contains("routing")) {
        const auto& r = j["routing"];
        cfg.routing.algorithm        = r.value("algorithm",       cfg.routing.algorithm);
        cfg.routing.preprocess_ch    = r.value("preprocess_ch",   cfg.routing.preprocess_ch);
        cfg.routing.reroute_threshold= r.value("reroute_threshold",cfg.routing.reroute_threshold);
        cfg.routing.cache_entries    = r.value("cache_entries",   cfg.routing.cache_entries);
        cfg.routing.use_traffic_costs   = r.value("use_traffic_costs",   cfg.routing.use_traffic_costs);
        cfg.routing.randomization_sigma = r.value("randomization_sigma", cfg.routing.randomization_sigma);
        if (r.contains("ch_cache"))
            cfg.routing.ch_cache = r["ch_cache"].get<std::string>();
    }
    // Cross-section: propagate randomization sigma to simulation config
    cfg.simulation.route_randomization_sigma = cfg.routing.randomization_sigma;

    // ── traffic ─────────────────────────────────────────────────────────────
    if (j.contains("traffic")) {
        const auto& t = j["traffic"];
        cfg.traffic.model          = t.value("model",          cfg.traffic.model);
    }

    // ── demand ──────────────────────────────────────────────────────────────
    if (j.contains("demand")) {
        const auto& d = j["demand"];
        cfg.demand.source       = d.value("source",       cfg.demand.source);
        cfg.demand.demand_scale = d.value("demand_scale", cfg.demand.demand_scale);
        if (d.contains("od_csv"))
            cfg.demand.od_csv = d["od_csv"].get<std::string>();
        if (d.contains("person_trips_csv"))
            cfg.demand.person_trips_csv = d["person_trips_csv"].get<std::string>();
        if (d.contains("maz_to_node_csv"))
            cfg.demand.maz_to_node_csv = d["maz_to_node_csv"].get<std::string>();
        // Mode selection
        if (d.contains("modes"))
            cfg.demand.modes = d["modes"].get<std::vector<std::string>>();
        if (d.contains("modes_split")) {
            cfg.demand.modes_split.clear();
            for (const auto& [k, v] : d["modes_split"].items())
                cfg.demand.modes_split[k] = v.get<float>();
        }

        if (d.contains("synthetic")) {
            const auto& s = d["synthetic"];
            auto& sp = cfg.demand.synthetic;
            sp.total_agents = s.value("total_agents", sp.total_agents);
            sp.beta         = s.value("beta",         sp.beta);
            sp.peak_mean_s  = s.value("peak_mean_s",  sp.peak_mean_s);
            sp.peak_std_s   = s.value("peak_std_s",   sp.peak_std_s);
            sp.seed         = s.value("seed",         sp.seed);
        }
    }

    // ── mode_choice ──────────────────────────────────────────────────────────
    if (j.contains("mode_choice")) {
        const auto& mc = j["mode_choice"];
        cfg.mode_choice.model           = mc.value("model",           cfg.mode_choice.model);
        cfg.mode_choice.multimodal      = mc.value("multimodal",      cfg.mode_choice.multimodal);
        cfg.mode_choice.walk_speed_ms   = mc.value("walk_speed_ms",   cfg.mode_choice.walk_speed_ms);
        cfg.mode_choice.bike_speed_ms   = mc.value("bike_speed_ms",   cfg.mode_choice.bike_speed_ms);
        cfg.mode_choice.max_walk_access_m= mc.value("max_walk_access_m",cfg.mode_choice.max_walk_access_m);
        cfg.mode_choice.vot_eur_per_hour = mc.value("vot_eur_per_hour",cfg.mode_choice.vot_eur_per_hour);
        if (mc.contains("mnl")) {
            const auto& mnl = mc["mnl"];
            auto& p = cfg.mode_choice.mnl;
            p.asc_car    = mnl.value("asc_car",    p.asc_car);
            p.asc_bike   = mnl.value("asc_bike",   p.asc_bike);
            p.asc_walk   = mnl.value("asc_walk",   p.asc_walk);
            p.asc_transit= mnl.value("asc_transit",p.asc_transit);
            p.beta_time  = mnl.value("beta_time",  p.beta_time);
            p.beta_cost  = mnl.value("beta_cost",  p.beta_cost);
            p.beta_wait  = mnl.value("beta_wait",  p.beta_wait);
            p.beta_walk  = mnl.value("beta_walk",  p.beta_walk);
        }
    }

    // ── output ───────────────────────────────────────────────────────────────
    if (j.contains("output")) {
        const auto& o = j["output"];
        if (o.contains("output_dir"))
            cfg.output.output_dir = o["output_dir"].get<std::string>();
        if (o.contains("writers"))
            cfg.output.writers = o["writers"].get<std::vector<std::string>>();
        cfg.output.snapshot_interval_s = o.value("snapshot_interval_s", cfg.output.snapshot_interval_s);
        cfg.output.store_trajectories  = o.value("store_trajectories",  cfg.output.store_trajectories);
        cfg.output.store_link_stats    = o.value("store_link_stats",    cfg.output.store_link_stats);
    }

    log_summary(cfg);
    return cfg;
}

// ── Save ──────────────────────────────────────────────────────────────────────
void ScenarioConfigIO::save(const ScenarioConfig& cfg,
                               const std::filesystem::path& json_path) {
    json j;
    j["name"]    = cfg.name;
    j["version"] = cfg.version;

    // Serialise as datetime strings so sim_date survives a round-trip.
    auto fmt_dt = [&](SimTime t) -> std::string {
        int h = static_cast<int>(t) / 3600;
        int m = (static_cast<int>(t) % 3600) / 60;
        int s = static_cast<int>(t) % 60;
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
        const std::string& date = cfg.sim_date.empty() ? "1970-01-01" : cfg.sim_date;
        return date + " " + buf;
    };
    j["simulation"]["start_time"]         = fmt_dt(cfg.simulation.start_time);
    j["simulation"]["end_time"]           = fmt_dt(cfg.simulation.end_time);
    j["simulation"]["sync_window_s"]      = cfg.simulation.sync_window_s;
    j["simulation"]["reroute_thresh"]     = cfg.simulation.reroute_thresh;
    j["simulation"]["reroute_interval_s"] = cfg.simulation.reroute_interval_s;
    j["simulation"]["num_threads"]        = cfg.simulation.num_threads;
    j["simulation"]["store_traces"]       = cfg.simulation.store_traces;
    j["simulation"]["traffic_model"]      = cfg.simulation.traffic_model;
    j["simulation"]["router"]             = cfg.simulation.router;
    j["simulation"]["snapshot_interval_s"]= cfg.simulation.snapshot_interval_s;

    j["network"]["osm_pbf"]          = cfg.network.osm_pbf.string();
    j["network"]["simplify_topology"]= cfg.network.simplify_topology;
    j["network"]["extract_transit"]  = cfg.network.extract_transit;
    if (cfg.network.gtfs_dir)
        j["network"]["gtfs_dir"]     = cfg.network.gtfs_dir->string();
    if (cfg.network.graph_bin)
        j["network"]["graph_bin"]    = cfg.network.graph_bin->string();

    j["routing"]["algorithm"]        = cfg.routing.algorithm;
    j["routing"]["preprocess_ch"]    = cfg.routing.preprocess_ch;
    j["routing"]["reroute_threshold"]= cfg.routing.reroute_threshold;
    j["routing"]["cache_entries"]    = cfg.routing.cache_entries;
    j["routing"]["use_traffic_costs"]= cfg.routing.use_traffic_costs;

    j["traffic"]["model"]           = cfg.traffic.model;
    j["demand"]["source"]           = cfg.demand.source;
    if (cfg.demand.od_csv)
        j["demand"]["od_csv"]       = cfg.demand.od_csv->string();

    j["output"]["output_dir"]          = cfg.output.output_dir.string();
    j["output"]["writers"]             = cfg.output.writers;
    j["output"]["snapshot_interval_s"] = cfg.output.snapshot_interval_s;

    std::ofstream f(json_path);
    f << j.dump(2);
}

// ── Validate ──────────────────────────────────────────────────────────────────
std::string ScenarioConfigIO::validate(const ScenarioConfig& cfg) {
    // graph_bin that already exists makes osm_pbf optional
    bool has_bin = cfg.network.graph_bin &&
                   std::filesystem::exists(*cfg.network.graph_bin);
    if (!has_bin) {
        if (cfg.network.osm_pbf.empty())
            return "network.osm_pbf is required (no graph_bin cache found)";
        if (!std::filesystem::exists(cfg.network.osm_pbf))
            return "network.osm_pbf does not exist: " + cfg.network.osm_pbf.string();
    }
    if (cfg.simulation.end_time <= cfg.simulation.start_time)
        return "simulation.end_time must be > start_time";
    if (cfg.demand.source == "od_csv" && !cfg.demand.od_csv)
        return "demand.od_csv is required when demand.source = od_csv";
    return {};
}

// ── Log summary ───────────────────────────────────────────────────────────────
void ScenarioConfigIO::log_summary(const ScenarioConfig& cfg) {
    spdlog::info("=== nomad scenario: {} ===", cfg.name);
    spdlog::info("  OSM:       {}", cfg.network.osm_pbf.string());
    spdlog::info("  Time:      {:.0f}s – {:.0f}s", cfg.simulation.start_time, cfg.simulation.end_time);
    spdlog::info("  Traffic:   {}", cfg.simulation.traffic_model);
    spdlog::info("  Router:    {}", cfg.simulation.router);
    spdlog::info("  Demand:    {}", cfg.demand.source);
    spdlog::info("  Threads:   {}", cfg.simulation.num_threads == 0 ? "auto" :
        std::to_string(cfg.simulation.num_threads));
    spdlog::info("  Output:    {}", cfg.output.output_dir.string());
    if (cfg.network.gtfs_dir)
        spdlog::info("  GTFS:      {}", cfg.network.gtfs_dir->string());
}

} // namespace nomad
