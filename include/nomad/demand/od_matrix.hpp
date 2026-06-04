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
// Departure times are sampled from a truncated normal distribution with
// mean=depart_mean_s and std=depart_std_s (minimum = depart_mean_s - 3σ).
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

    // Node-based construction (no zone lookup needed)
    explicit OdMatrixDemand(std::vector<OdEntry> entries, uint32_t seed = 42);

    // Load from CSV file
    static OdMatrixDemand from_csv(const std::filesystem::path& csv_path,
                                    uint32_t seed = 42);

    std::size_t generate(const Graph&, EventQueue&,
                          AgentColdStore&, RouteStore&) override;

    std::string_view model_name() const override { return "od_matrix"; }

private:
    SimTime sample_departure(const OdEntry& e);

    std::vector<OdEntry> entries_;
    std::mt19937         rng_;
};

} // namespace nomad
