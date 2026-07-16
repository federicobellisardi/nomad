#pragma once

#include <nomad/core/simulation.hpp>
#include <nomad/core/types.hpp>
#include <nomad/demand/mode_choice.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace nomad {

// ── JSON scenario configuration ───────────────────────────────────────────────
// All simulation parameters are controlled from a single JSON file.
// Example: nomad_cli --config scenario.json
//
// Full schema documented in data/schemas/scenario_schema.json.
// Every field has a sensible default; only non-default values need to appear.

struct NetworkConfig {
    std::filesystem::path osm_pbf;          // required unless graph_bin exists
    std::filesystem::path osm_tag_config;   // optional: YAML tag rules
    bool simplify_topology = true;
    bool extract_transit   = true;          // extract bus stops / railway
    std::optional<std::filesystem::path> gtfs_dir;   // GTFS feed for PT
    std::optional<std::filesystem::path> graph_bin;  // binary graph cache (load if exists, save after build)

    // Bounding box filter (optional; default = full file)
    struct BBox { float min_lon, min_lat, max_lon, max_lat; };
    std::optional<BBox> bbox;
};

struct RoutingConfig {
    std::string algorithm = "CH";    // "CH" | "astar"
    bool preprocess_ch    = true;
    std::optional<std::filesystem::path> ch_cache; // load/save preprocessed CH
    float reroute_threshold = 0.25f;
    std::size_t cache_entries = 500'000;
    bool  use_traffic_costs      = true;   // dynamic costs from traffic model
    // Stochastic pre-routing: fraction sigma of agents use A* with log-normal
    // road-class cost perturbation instead of deterministic CH.
    // sigma = std-dev of Normal before exp(); 0 = disabled (pure CH).
    // Recommended: 0.2–0.4 for cities with heavy motorway concentration.
    float randomization_sigma    = 0.0f;
};

struct TrafficConfig {
    std::string model = "queue";     // "queue" | "ltm"
};

struct DemandConfig {
    std::string source = "od_csv";   // "od_csv" | "gravity" | "radiation" | "activity_plan"

    // ── Mode selection ─────────────────────────────────────────────────────────
    // "modes": ["car"]                → tutti gli agenti usano l'auto
    // "modes": ["car","walk","bike"]  → ogni agente sceglie in base a modes_split
    // "modes": ["car","walk","bike","transit"] → multimodale completo
    std::vector<std::string> modes = {"car"};

    // Proporzioni per modalità (somma = 1.0). Usato solo se modes ha > 1 elemento.
    // Se la voce manca, la distribuzione è uniforme tra i modi elencati.
    std::map<std::string, float> modes_split = {{"car", 1.0f}};

    // od_csv
    std::optional<std::filesystem::path> od_csv;

    // gravity / radiation
    struct SyntheticParams {
        uint32_t total_agents = 10000;
        float beta            = 0.003f;
        float peak_mean_s     = 28800.0f;
        float peak_std_s      = 3600.0f;
        uint32_t seed         = 42;
    };
    SyntheticParams synthetic;

    // activity_plan (ActivitySim)
    std::optional<std::filesystem::path> person_trips_csv;
    std::optional<std::filesystem::path> maz_to_node_csv;

    // Demand scaling: multiply all OD counts by this factor [0, 1].
    // Use to calibrate against observed traffic counts or correct for modal-share
    // uncertainty in raw mobility data (e.g. MITMA).
    float demand_scale = 1.0f;
};

struct ModeChoiceConfig {
    std::string model = "MNL";       // "MNL" | "from_plan"
    MnlModeChoice::Params mnl;
    float vot_eur_per_hour = 12.0f;
    bool multimodal = true;          // allow mixed-mode trips
    float walk_speed_ms = 1.39f;     // 5 km/h
    float bike_speed_ms = 4.17f;     // 15 km/h
    float max_walk_access_m = 800.0f; // max walk distance to PT stop
};

struct OutputConfig {
    std::filesystem::path output_dir = "nomad_output";
    std::vector<std::string> writers = {"geojson"};  // "geojson" | "parquet" | "csv"
    uint32_t snapshot_interval_s = 300;
    bool store_trajectories = false; // full per-agent trajectories (large)
    bool store_link_stats   = true;
    bool store_activities   = false;
};

// ── Top-level scenario configuration ─────────────────────────────────────────
struct ScenarioConfig {
    std::string name    = "nomad_scenario";
    std::string version = "0.1";
    std::string sim_date;  // date portion of start_time ("YYYY-MM-DD"), empty if not set

    SimulationConfig simulation;
    NetworkConfig    network;
    RoutingConfig    routing;
    TrafficConfig    traffic;
    DemandConfig     demand;
    ModeChoiceConfig mode_choice;
    OutputConfig     output;
};

// ── Utilities ─────────────────────────────────────────────────────────────────
inline AgentMode str_to_mode(const std::string& s) {
    if (s == "car"     || s == "Car")     return AgentMode::Car;
    if (s == "bike"    || s == "Bike")    return AgentMode::Bike;
    if (s == "walk"    || s == "Walk")    return AgentMode::Walk;
    if (s == "transit" || s == "Transit") return AgentMode::Transit;
    return AgentMode::Car;
}

// ── JSON loader/saver ─────────────────────────────────────────────────────────
class ScenarioConfigIO {
public:
    // Load from JSON file (fields not present use their defaults)
    static ScenarioConfig load(const std::filesystem::path& json_path);

    // Save to JSON file (round-trippable)
    static void save(const ScenarioConfig& cfg,
                      const std::filesystem::path& json_path);

    // Validate: returns empty string on success, error message on failure
    static std::string validate(const ScenarioConfig& cfg);

    // Print a summary of the config to spdlog
    static void log_summary(const ScenarioConfig& cfg);
};

} // namespace nomad
