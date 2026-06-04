#pragma once

#include <nomad/demand/demand_model.hpp>

#include <filesystem>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── ActivitySim demand connector ─────────────────────────────────────────────
// Reads ActivitySim output and maps it to nomad agent activity plans.
//
// Supported ActivitySim output formats:
//   person_trips.csv — columns: person_id, origin_maz, dest_maz, purpose,
//                               depart_period, trip_mode
//   tours.csv        — optional; used to reconstruct full daily activity chains
//
// MAZ→NodeId mapping:
//   maz_to_node.csv  — columns: maz_id, node_id
//   (generated externally by spatial join of MAZ centroids to graph nodes)
//
// Mode mapping (ActivitySim → AgentMode):
//   DRIVEALONE, SR2, SR3+ → Car
//   WALK_LOC, WALK_EXP    → Transit
//   BIKE                  → Bike
//   WALK                  → Walk
//
// Departure period to departure time:
//   ActivitySim uses periods (1=3AM-6AM, 2=6AM-9AM, ...). We sample
//   uniformly within the period and add Gaussian jitter (σ=10min).
class ActivityPlanDemand final : public IDemandModel {
public:
    explicit ActivityPlanDemand(
        const std::filesystem::path& person_trips_csv,
        const std::filesystem::path& maz_to_node_csv);

    std::size_t generate(const Graph&, EventQueue&,
                          AgentColdStore&, RouteStore&) override;

    std::string_view model_name() const override { return "activity_plan"; }

private:
    struct RawTrip {
        uint64_t  person_id;
        uint32_t  origin_maz;
        uint32_t  dest_maz;
        uint8_t   purpose;
        uint8_t   depart_period;
        AgentMode mode;
    };

    static AgentMode map_mode(const std::string& trip_mode);
    static SimTime   period_to_time(uint8_t period);

    std::vector<RawTrip>                       trips_;
    std::unordered_map<uint32_t, NodeId>       maz_to_node_;
};

} // namespace nomad
