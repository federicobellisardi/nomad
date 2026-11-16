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
    : entries_(std::move(entries)), rng_(seed), seed_(seed), time_min_s_(time_min_s) {}

namespace {
// Uniform [0,1) from a hash of (seed, CSV row, trip index, stream).
double trip_uniform(uint32_t seed, uint32_t row, uint32_t i, uint64_t stream) {
    uint64_t x = (static_cast<uint64_t>(seed) << 32) ^ (static_cast<uint64_t>(row) * 0x9E3779B97F4A7C15ull)
                 ^ (static_cast<uint64_t>(i) * 0xC2B2AE3D27D4EB4Full) ^ (stream * 0x165667B19E3779F9ull);
    for (int k = 0; k < 2; ++k) {   // splitmix64 finaliser, twice
        x += 0x9E3779B97F4A7C15ull;
        uint64_t z = x;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        x = z ^ (z >> 31);
    }
    return (x >> 11) * (1.0 / 9007199254740992.0);
}
} // namespace

OdMatrixDemand OdMatrixDemand::from_csv(const std::filesystem::path& csv_path,
                                          uint32_t seed,
                                          float time_min_s,
                                          float time_max_s,
                                          const std::vector<AgentMode>& allowed_modes,
                                          float demand_scale,
                                          bool nested_sampling) {
    std::ifstream f(csv_path);
    if (!f) throw std::runtime_error("Cannot open OD CSV: " + csv_path.string());

    std::vector<OdEntry> entries;
    std::string line;
    std::getline(f, line); // skip header

    uint32_t row = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const uint32_t this_row = row++;
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
        e.row           = this_row;

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
    if (nested_sampling) {
        // counts stay full; trips are drawn per trip in generate()
        for (const auto& en : entries) n_agents_raw += en.count;
    } else if (demand_scale < 1.0f && demand_scale > 0.0f) {
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
    if (nested_sampling)
        spdlog::info("OdMatrixDemand: nested sampling, demand_scale={:.2f} of {} trips "
                     "(drawn per trip at generation)", demand_scale, n_agents_raw);
    else if (demand_scale < 1.0f)
        spdlog::info("OdMatrixDemand: demand_scale={:.2f} → {} → {} agents",
                      demand_scale, n_agents_raw, n_agents_loaded);
    spdlog::info("OdMatrixDemand: loaded {} entries ({} agents) from {}",
                  entries.size(), n_agents_loaded, csv_path.string());
    for (int h = 0; h < 25; ++h)
        if (hour_counts[static_cast<std::size_t>(h)] > 0)
            spdlog::info("  {:02d}:00  {:6d} agents", h, hour_counts[static_cast<std::size_t>(h)]);

    OdMatrixDemand dm{std::move(entries), seed, time_min_s};
    if (nested_sampling) dm.set_nested_sampling(demand_scale);
    return dm;
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
    auto kept = [&](const OdEntry& en, uint32_t i) {
        return !nested_ || trip_uniform(seed_, en.row, i, 1) < static_cast<double>(nested_scale_);
    };
    std::size_t total = 0;
    for (const auto& entry : entries_)
        for (uint32_t i = 0; i < entry.count; ++i) total += kept(entry, i) ? 1 : 0;

    cold.resize(total);
    routes.resize(total);

    AgentId aid = 0;
    for (const auto& entry : entries_) {
        for (uint32_t i = 0; i < entry.count; ++i) {
            if (!kept(entry, i)) continue;
            SimTime depart;
            if (nested_) {
                const double start = std::max(entry.depart_mean_s, time_min_s_);
                const double end   = std::max(static_cast<double>(entry.depart_std_s), start);
                depart = start + trip_uniform(seed_, entry.row, i, 2) * (end - start);
            } else {
                depart = sample_departure(entry);
            }

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
            ++aid;
        }
    }

    spdlog::info("OdMatrixDemand: generated {} agents", total);
    return total;
}

} // namespace nomad
