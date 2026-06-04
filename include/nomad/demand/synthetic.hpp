#pragma once

#include <nomad/demand/demand_model.hpp>

#include <filesystem>
#include <random>
#include <vector>

namespace nomad {

// ── Gravity model demand ──────────────────────────────────────────────────────
// Generates synthetic OD demand using a spatial interaction (gravity) model.
// Trip volumes T_ij are proportional to:
//   T_ij ∝ P_i × A_j × exp(-β × d_ij)
// where P_i is the production potential of origin zone i (e.g., population),
// A_j is the attraction potential of destination zone j (e.g., employment),
// and d_ij is the network distance or Euclidean proxy.
//
// Input: node scores from Graph::nodes[i].score (loaded from OSM or CSV),
//        or a population raster (GeoTIFF via GDAL, optional dependency).
//
// This model is suitable when no empirical OD data is available. For
// calibrated scenarios, prefer OdMatrixDemand.
class GravityDemand final : public IDemandModel {
public:
    struct Config {
        uint32_t total_agents     = 10000;
        float    beta             = 0.003f;  // distance decay parameter [1/m]
        float    peak_mean_s      = 28800.0f; // 8:00 AM
        float    peak_std_s       = 3600.0f;  // 1h standard deviation
        AgentMode mode            = AgentMode::Car;
        uint32_t seed             = 42;
        // Optional: fraction of trips for each purpose (home→work, home→shop, etc.)
        float    work_fraction    = 0.6f;
        float    shop_fraction    = 0.2f;
        float    leisure_fraction = 0.2f;
    };

    explicit GravityDemand();
    explicit GravityDemand(Config cfg);

    std::size_t generate(const Graph&, EventQueue&,
                          AgentColdStore&, RouteStore&) override;

    std::string_view model_name() const override { return "gravity"; }

private:
    // Build trip table: returns (origin, destination) pairs
    std::vector<std::pair<NodeId, NodeId>>
    build_trip_table(const Graph& g);

    Config       cfg_;
    std::mt19937 rng_;
};

// ── Radiation model demand ────────────────────────────────────────────────────
// Alternative synthetic demand using the radiation model (Simini et al. 2012),
// which requires only population data and has no free parameters.
// T_ij ∝ P_i × P_j / ((P_i + s_ij) × (P_i + P_j + s_ij))
// where s_ij = total population within circle of radius d_ij (excluding i,j).
class RadiationDemand final : public IDemandModel {
public:
    struct Config {
        uint32_t  total_agents = 10000;
        float     peak_mean_s  = 28800.0f;
        float     peak_std_s   = 3600.0f;
        AgentMode mode         = AgentMode::Car;
        uint32_t  seed         = 42;
    };

    explicit RadiationDemand();
    explicit RadiationDemand(Config cfg);

    std::size_t generate(const Graph&, EventQueue&,
                          AgentColdStore&, RouteStore&) override;

    std::string_view model_name() const override { return "radiation"; }

private:
    Config       cfg_;
    std::mt19937 rng_;
};

} // namespace nomad
