#pragma once

#include <nomad/core/agent.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <cstdint>
#include <vector>

namespace nomad {

// ── Temporal aggregator ───────────────────────────────────────────────────────
// Accumulates link counts over a time window and computes summary statistics.
// Used by output writers and the Python analysis layer.
struct LinkTimeSlice {
    SimTime  time_start;
    SimTime  time_end;
    EdgeId   edge;
    uint32_t count;
    float    avg_travel_time_s;
    float    avg_speed_ms;
    float    volume_capacity_ratio;
};

class TemporalAggregator {
public:
    explicit TemporalAggregator(float bin_width_s = 300.0f);

    void record(EdgeId e, SimTime enter_time, SimTime exit_time,
                 float travel_time_s);

    // Flush all complete bins (those with end_time ≤ up_to_time)
    std::vector<LinkTimeSlice> flush(SimTime up_to_time,
                                      const Graph& graph);

private:
    struct Bin {
        uint32_t count{0};
        float    sum_travel_time{0.0f};
    };

    float bin_width_;
    // bins_[bin_index][edge_id]
    std::vector<std::vector<Bin>> bins_;
    SimTime first_bin_start_{0.0};
};

// ── Spatial aggregator ────────────────────────────────────────────────────────
// Counts agents within geographic zones (TAZ polygons or grid cells).
// Useful for density maps and zone-level statistics.
struct ZoneSnapshot {
    ZoneId   zone;
    SimTime  time;
    uint32_t count;
    float    avg_speed_ms;
};

} // namespace nomad
