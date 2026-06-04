#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <cmath>
#include <vector>

namespace nomad {

// ── GEH statistic ─────────────────────────────────────────────────────────────
// Standard transport calibration metric (Geoffrey E. Havers, 1983).
// Combines relative and absolute error; values < 5 indicate good calibration.
// GEH < 5 is the standard acceptance criterion for microsimulation validation.
inline float geh(float simulated, float observed) noexcept {
    if (observed < 1e-6f && simulated < 1e-6f) return 0.0f;
    float diff = simulated - observed;
    float mean = 0.5f * (simulated + observed);
    return std::sqrt(diff * diff / mean);
}

// ── Level of service thresholds (HCM 6th edition) ────────────────────────────
enum class LevelOfService : uint8_t { A=0, B, C, D, E, F };

inline LevelOfService volume_capacity_to_los(float vc_ratio) noexcept {
    if (vc_ratio < 0.30f) return LevelOfService::A;
    if (vc_ratio < 0.55f) return LevelOfService::B;
    if (vc_ratio < 0.75f) return LevelOfService::C;
    if (vc_ratio < 0.90f) return LevelOfService::D;
    if (vc_ratio < 1.00f) return LevelOfService::E;
    return LevelOfService::F;
}

// ── Travel time ratio ─────────────────────────────────────────────────────────
// Ratio of actual to free-flow travel time. Used for congestion detection.
inline float tt_ratio(float actual_s, float freeflow_s) noexcept {
    return freeflow_s > 0.0f ? actual_s / freeflow_s : 1.0f;
}

// ── Network-wide performance indicators ──────────────────────────────────────
struct NetworkPerformance {
    float total_vehicle_km;       // VKT
    float total_vehicle_hours;    // VHT
    float average_speed_kmh;      // VKT / VHT
    float congestion_index;       // VHT_congested / VHT_freeflow - 1
    float fraction_congested;     // fraction of edges with vc_ratio > 0.9
};

NetworkPerformance compute_performance(const Graph&          graph,
                                        const ITrafficModel&  traffic);

// ── Accessibility indicator ───────────────────────────────────────────────────
// Cumulative opportunities within a travel time threshold.
// accessibility(node, threshold) = number of nodes reachable within threshold seconds.
// Uses a pre-computed distance matrix or a one-to-all Dijkstra pass.
struct AccessibilityResult {
    NodeId node;
    float  threshold_s;
    uint32_t reachable_nodes;
    float    avg_travel_time_s;
};

// ── Validation helpers ────────────────────────────────────────────────────────
struct CountComparison {
    EdgeId edge;
    float  simulated;
    float  observed;
    float  geh_value;
    LevelOfService simulated_los;
};

// Compare simulated link volumes against observed loop counter data.
// observed_counts: pairs of (edge_id, observed_count_veh_h)
std::vector<CountComparison>
compare_counts(const ITrafficModel&                           traffic,
               const std::vector<std::pair<EdgeId, float>>&  observed_counts);

// Fraction of comparisons with GEH < threshold (default 5)
float geh_pass_rate(const std::vector<CountComparison>& comparisons,
                     float threshold = 5.0f);

} // namespace nomad
