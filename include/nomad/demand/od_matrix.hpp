#pragma once

#include <nomad/demand/demand_model.hpp>

#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace nomad {

// ── OD matrix demand ─────────────────────────────────────────────────────────
// Reads an origin-destination matrix from CSV or Parquet and creates agents.
// Each row defines a batch of trips sharing origin zone, destination zone,
// mode, and departure time distribution.
//
// CSV format (header required):
//   origin_node, dest_node, count, mode, depart_mean_s, depart_std_s
//
// Zone-based format (with zone_nodes map):
//   origin_zone, dest_zone, count, mode, depart_mean_s, depart_std_s
// Agents are distributed uniformly across all nodes in each zone.
//
// Departure times are sampled from a Uniform distribution over the interval
// [depart_mean_s, depart_std_s] (window start / window end in seconds from
// midnight). This matches hourly MITMA buckets: build_od.py writes
// depart_mean_s = hour * 3600, depart_std_s = (hour+1) * 3600.
class OdMatrixDemand final : public IDemandModel {
public:
    struct OdEntry {
        NodeId    origin_node;    // kInvalidNode if zone-based
        NodeId    dest_node;      // kInvalidNode if zone-based
        ZoneId    origin_zone;    // kInvalidZone if node-based
        ZoneId    dest_zone;      // kInvalidZone if node-based
        uint32_t  count;
        AgentMode mode;
        float     depart_mean_s;
        float     depart_std_s;
    };

    // Node-based construction (no zone lookup needed).
    // time_min_s is used to clamp departure times of partially-overlapping hour buckets.
    explicit OdMatrixDemand(std::vector<OdEntry> entries, uint32_t seed = 42,
                             float time_min_s = -1e9f);

    // Load from CSV file.
    // Entries whose mean departure time falls outside [time_min_s - 3σ, time_max_s]
    // are skipped. If allowed_modes is non-empty, only matching modes are loaded.
    // demand_scale ∈ (0,1]: multiply each OD pair count by this factor.
    static OdMatrixDemand from_csv(const std::filesystem::path& csv_path,
                                    uint32_t seed = 42,
                                    float time_min_s = -1e9f,
                                    float time_max_s =  1e9f,
                                    const std::vector<AgentMode>& allowed_modes = {},
                                    float demand_scale = 1.0f);

    std::size_t generate(const Graph&, EventQueue&,
                          AgentColdStore&, RouteStore&) override;

    std::string_view model_name() const override { return "od_matrix"; }

private:
    SimTime sample_departure(const OdEntry& e);

    std::vector<OdEntry> entries_;
    std::mt19937         rng_;
    float                time_min_s_{-1e9f};  // lower bound for departure clamp
};

} // namespace nomad
