#include <nomad/demand/od_matrix.hpp>
#include <nomad/demand/departure_sampler.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>

namespace nomad {

OdMatrixDemand::OdMatrixDemand(std::vector<OdEntry> entries, uint32_t seed)
    : entries_(std::move(entries)), rng_(seed) {}

OdMatrixDemand OdMatrixDemand::from_csv(const std::filesystem::path& csv_path,
                                          uint32_t seed) {
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

        e.origin_node   = static_cast<NodeId>(origin);
        e.dest_node     = static_cast<NodeId>(dest);
        e.count         = count;
        e.depart_mean_s = mean_s;
        e.depart_std_s  = std_s;

        if (mode_str == "car" || mode_str == "Car") e.mode = AgentMode::Car;
        else if (mode_str == "walk" || mode_str == "Walk") e.mode = AgentMode::Walk;
        else if (mode_str == "bike" || mode_str == "Bike") e.mode = AgentMode::Bike;
        else if (mode_str == "transit" || mode_str == "Transit") e.mode = AgentMode::Transit;
        else e.mode = AgentMode::Car;

        entries.push_back(e);
    }

    spdlog::info("OdMatrixDemand: loaded {} OD entries from {}", entries.size(), csv_path.string());
    return OdMatrixDemand{std::move(entries), seed};
}

SimTime OdMatrixDemand::sample_departure(const OdEntry& e) {
    DepartureSampler sampler = DepartureSampler::lognormal(e.depart_mean_s, e.depart_std_s);
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
