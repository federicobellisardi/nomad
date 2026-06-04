#include <nomad/analytics/aggregator.hpp>
#include <nomad/analytics/metrics.hpp>
#include <algorithm>
#include <cmath>

namespace nomad {

TemporalAggregator::TemporalAggregator(float bin_width_s)
    : bin_width_(bin_width_s) {}

void TemporalAggregator::record(EdgeId e, SimTime /*enter*/, SimTime /*exit*/,
                                  float travel_time_s) {
    int bin = static_cast<int>(travel_time_s / bin_width_);
    if (bin >= static_cast<int>(bins_.size()))
        bins_.resize(bin + 1);
    if (e >= bins_[bin].size())
        bins_[bin].resize(e + 1);
    bins_[bin][e].count++;
    bins_[bin][e].sum_travel_time += travel_time_s;
}

std::vector<LinkTimeSlice> TemporalAggregator::flush(SimTime up_to_time, const Graph& g) {
    std::vector<LinkTimeSlice> result;
    int max_bin = static_cast<int>(up_to_time / bin_width_);
    for (int b = 0; b < max_bin && b < static_cast<int>(bins_.size()); ++b) {
        for (uint32_t e = 0; e < bins_[b].size(); ++e) {
            const Bin& bin = bins_[b][e];
            if (bin.count == 0) continue;
            LinkTimeSlice s{};
            s.time_start       = first_bin_start_ + b * bin_width_;
            s.time_end         = s.time_start + bin_width_;
            s.edge             = e;
            s.count            = bin.count;
            s.avg_travel_time_s= bin.sum_travel_time / bin.count;
            float len = (e < g.num_edges()) ? g.edges[e].length_m : 0.0f;
            s.avg_speed_ms     = (s.avg_travel_time_s > 0) ? len / s.avg_travel_time_s : 0.0f;
            s.volume_capacity_ratio = (e < g.num_edges() && g.edges[e].capacity > 0)
                ? (bin.count * 3600.0f / bin_width_) / g.edges[e].capacity : 0.0f;
            result.push_back(s);
        }
        bins_[b].clear();
    }
    return result;
}

// ── NetworkPerformance ────────────────────────────────────────────────────────
NetworkPerformance compute_performance(const Graph& g, const ITrafficModel& traffic) {
    const auto states = traffic.link_states();
    NetworkPerformance p{};
    uint32_t congested = 0;
    float total_freeflow_vht = 0.0f;

    for (uint32_t e = 0; e < g.num_edges(); ++e) {
        float occ = states[e].occupancy.load();
        if (occ == 0.0f) continue;
        float len  = g.edges[e].length_m;
        float tt   = traffic.current_travel_time(e);
        float ff   = g.free_flow_time(e);
        float cap  = g.edges[e].capacity;
        p.total_vehicle_km    += occ * len / 1000.0f;
        p.total_vehicle_hours += occ * tt / 3600.0f;
        total_freeflow_vht    += occ * ff / 3600.0f;
        if (cap > 0 && occ * 3600.0f / (tt > 0 ? tt : 1.0f) / cap > 0.9f) ++congested;
    }

    p.average_speed_kmh = (p.total_vehicle_hours > 0)
        ? p.total_vehicle_km / p.total_vehicle_hours : 0.0f;
    p.congestion_index  = (total_freeflow_vht > 0)
        ? (p.total_vehicle_hours / total_freeflow_vht) - 1.0f : 0.0f;
    p.fraction_congested= (g.num_edges() > 0)
        ? static_cast<float>(congested) / g.num_edges() : 0.0f;
    return p;
}

// ── Count comparisons ─────────────────────────────────────────────────────────
std::vector<CountComparison>
compare_counts(const ITrafficModel& traffic,
               const std::vector<std::pair<EdgeId, float>>& observed) {
    std::vector<CountComparison> result;
    result.reserve(observed.size());
    const auto states = traffic.link_states();
    for (const auto& [eid, obs] : observed) {
        float sim = states[eid].inflow_rate.load() * 3600.0f;
        result.push_back({eid, sim, obs, geh(sim, obs),
            volume_capacity_to_los(obs > 0 ? sim / obs : 0.0f)});
    }
    return result;
}

float geh_pass_rate(const std::vector<CountComparison>& comparisons, float threshold) {
    if (comparisons.empty()) return 0.0f;
    float pass = 0.0f;
    for (const auto& c : comparisons) if (c.geh_value < threshold) ++pass;
    return pass / comparisons.size();
}

} // namespace nomad
