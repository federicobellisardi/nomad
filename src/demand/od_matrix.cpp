#include <nomad/demand/od_matrix.hpp>
#include <nomad/demand/departure_sampler.hpp>
#include <nomad/config/scenario_config.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>

namespace nomad {

OdMatrixDemand::OdMatrixDemand(std::vector<OdEntry> entries, uint32_t seed,
                                float time_min_s)
    : entries_(std::move(entries)), rng_(seed), time_min_s_(time_min_s) {}

OdMatrixDemand OdMatrixDemand::from_csv(const std::filesystem::path& csv_path,
                                          uint32_t seed,
                                          float time_min_s,
                                          float time_max_s,
                                          const std::vector<AgentMode>& allowed_modes,
                                          float demand_scale) {
    std::ifstream f(csv_path);
    if (!f) throw std::runtime_error("Cannot open OD CSV: " + csv_path.string());

    std::vector<OdEntry> entries;
    std::string line;
    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        OdEntry e{};
        e.origin_zone = kInvalidZone;
        e.dest_zone   = kInvalidZone;

        auto next = [&](auto& out) -> bool {
            if (!std::getline(ss, tok, ',')) return false;
            if constexpr (std::is_same_v<std::remove_reference_t<decltype(out)>, std::string>) {
                out = tok;
            } else {
                auto r = std::from_chars(tok.data(), tok.data() + tok.size(), out);
                return r.ec == std::errc();
            }
            return true;
        };

        uint32_t origin = 0, dest = 0, count = 0;
        float mean_s = 0, std_s = 0;
        std::string mode_str;

        if (!next(origin) || !next(dest) || !next(count) ||
            !next(mode_str) || !next(mean_s) || !next(std_s)) {
            spdlog::warn("OD CSV: malformed row, skipping: {}", line);
            continue;
        }

        // Skip entries outside the simulation window.
        // depart_mean_s = window_start, depart_std_s = window_end (Uniform semantics).
        // An entry is irrelevant if its entire departure window falls outside [time_min_s, time_max_s].
        if (std_s  < time_min_s) continue;   // window ends before sim start
        if (mean_s > time_max_s) continue;   // window starts after sim end

        e.origin_node   = static_cast<NodeId>(origin);
        e.dest_node     = static_cast<NodeId>(dest);
        e.count         = count;
        e.depart_mean_s = mean_s;
        e.depart_std_s  = std_s;

        e.mode = str_to_mode(mode_str);

        if (!allowed_modes.empty()) {
            bool ok = false;
            for (auto m : allowed_modes) if (m == e.mode) { ok = true; break; }
            if (!ok) continue;
        }

        entries.push_back(e);
    }

    // Apply demand scaling via binomial thinning (preserves Poisson statistics).
    // Each trip in an OD pair independently survives with probability demand_scale.
    uint32_t n_agents_raw = 0;
    if (demand_scale < 1.0f && demand_scale > 0.0f) {
        std::mt19937 scale_rng(seed ^ 0xDEADBEEF);
        for (auto& en : entries) {
            n_agents_raw += en.count;
            std::binomial_distribution<uint32_t> binom(en.count, demand_scale);
            en.count = binom(scale_rng);
        }
        entries.erase(
            std::remove_if(entries.begin(), entries.end(),
                           [](const OdEntry& e) { return e.count == 0; }),
            entries.end());
    }

    // Hourly profile diagnostic
    std::array<uint32_t, 25> hour_counts{};
    uint32_t n_agents_loaded = 0;
    for (const auto& en : entries) {
        int h = static_cast<int>(en.depart_mean_s / 3600.0f);
        if (h >= 0 && h < 25) hour_counts[static_cast<std::size_t>(h)] += en.count;
        n_agents_loaded += en.count;
    }
    if (demand_scale < 1.0f)
        spdlog::info("OdMatrixDemand: demand_scale={:.2f} → {} → {} agents",
                      demand_scale, n_agents_raw, n_agents_loaded);
    spdlog::info("OdMatrixDemand: loaded {} entries ({} agents) from {}",
                  entries.size(), n_agents_loaded, csv_path.string());
    for (int h = 0; h < 25; ++h)
        if (hour_counts[static_cast<std::size_t>(h)] > 0)
            spdlog::info("  {:02d}:00  {:6d} agents", h, hour_counts[static_cast<std::size_t>(h)]);

    return OdMatrixDemand{std::move(entries), seed, time_min_s};
}

SimTime OdMatrixDemand::sample_departure(const OdEntry& e) {
    // depart_mean_s = window start, depart_std_s = window end (Uniform over MITMA hour bucket).
    // Clamp window start to time_min_s_ so agents in a partially-overlapping bucket
    // (e.g. periodo 5, window [18000,21600)) never depart before the simulation begins.
    float start = std::max(e.depart_mean_s, time_min_s_);
    DepartureSampler sampler = DepartureSampler::uniform(start, e.depart_std_s);
    return sampler.sample(rng_);
}

std::size_t OdMatrixDemand::generate(const Graph& /*graph*/,
                                       EventQueue& eq,
                                       AgentColdStore& cold,
                                       RouteStore& routes) {
    std::size_t total = 0;
    for (const auto& entry : entries_) total += entry.count;

    cold.resize(total);
    routes.resize(total);

    AgentId aid = 0;
    for (const auto& entry : entries_) {
        for (uint32_t i = 0; i < entry.count; ++i, ++aid) {
            SimTime depart = sample_departure(entry);

            // Create single activity: origin → destination
            ActivityPlan plan;
            plan.preferred_mode = entry.mode;
            plan.activities.push_back({entry.origin_node, depart, 0.0, ActivityType::Other});
            plan.activities.push_back({entry.dest_node, depart + 3600.0, 0.0, ActivityType::Other});
            cold.plans[aid] = std::move(plan);

            // Schedule departure event
            Event ev{};
            ev.time    = depart;
            ev.agent   = aid;
            ev.type    = EventType::AgentDepart;
            ev.payload = static_cast<uint32_t>(entry.origin_node);
            eq.push(ev);
        }
    }

    spdlog::info("OdMatrixDemand: generated {} agents", total);
    return total;
}

} // namespace nomad
