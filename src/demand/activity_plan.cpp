#include <nomad/demand/activity_plan.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <random>
#include <spdlog/spdlog.h>

namespace nomad {

ActivityPlanDemand::ActivityPlanDemand(
    const std::filesystem::path& trips_csv,
    const std::filesystem::path& maz_csv)
{
    // Load MAZ→Node mapping
    {
        std::ifstream f(maz_csv);
        if (!f) throw std::runtime_error("Cannot open MAZ mapping: " + maz_csv.string());
        std::string line;
        std::getline(f, line); // header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string maz_str, node_str;
            std::getline(ss, maz_str, ',');
            std::getline(ss, node_str, ',');
            if (!maz_str.empty() && !node_str.empty())
                maz_to_node_[static_cast<uint32_t>(std::stoul(maz_str))] =
                    static_cast<NodeId>(std::stoul(node_str));
        }
    }

    // Load person trips
    {
        std::ifstream f(trips_csv);
        if (!f) throw std::runtime_error("Cannot open trips CSV: " + trips_csv.string());
        std::string line;
        std::getline(f, line); // header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            std::string tok;
            RawTrip trip{};
            auto rd = [&]{ std::getline(ss, tok, ','); return tok; };
            try {
                trip.person_id      = std::stoull(rd());
                trip.origin_maz     = std::stoul(rd());
                trip.dest_maz       = std::stoul(rd());
                rd(); // purpose string → ignored in Phase 1
                trip.depart_period  = static_cast<uint8_t>(std::stoul(rd()));
                trip.mode           = map_mode(rd());
                trips_.push_back(trip);
            } catch (...) {
                spdlog::warn("ActivityPlanDemand: skipping malformed row");
            }
        }
    }
    spdlog::info("ActivityPlanDemand: {} trips, {} MAZ entries", trips_.size(), maz_to_node_.size());
}

AgentMode ActivityPlanDemand::map_mode(const std::string& s) {
    if (s == "DRIVEALONE" || s == "SR2" || s == "SR3PLUS") return AgentMode::Car;
    if (s == "WALK_LOC" || s == "WALK_EXP") return AgentMode::Transit;
    if (s == "BIKE") return AgentMode::Bike;
    if (s == "WALK") return AgentMode::Walk;
    return AgentMode::Car;
}

SimTime ActivityPlanDemand::period_to_time(uint8_t period) {
    // ActivitySim periods: 1=EA(3-6AM), 2=AM(6-9AM), 3=MD(9AM-3PM), 4=PM(3-7PM), 5=EV(7PM-3AM)
    static const float period_starts[] = {0,  3*3600, 6*3600, 9*3600, 15*3600, 19*3600};
    static const float period_ends[]   = {0,  6*3600, 9*3600, 15*3600, 19*3600, 27*3600};
    if (period < 1 || period > 5) return 28800.0f;
    float a = period_starts[period], b = period_ends[period];
    return a + (b - a) * 0.5f; // midpoint; production would sample uniformly
}

std::size_t ActivityPlanDemand::generate(const Graph& /*graph*/, EventQueue& eq,
                                           AgentColdStore& cold, RouteStore& routes) {
    cold.resize(trips_.size());
    routes.resize(trips_.size());

    for (std::size_t i = 0; i < trips_.size(); ++i) {
        const auto& trip = trips_[i];
        auto origin_it = maz_to_node_.find(trip.origin_maz);
        auto dest_it   = maz_to_node_.find(trip.dest_maz);
        if (origin_it == maz_to_node_.end() || dest_it == maz_to_node_.end()) continue;

        SimTime t = period_to_time(trip.depart_period);
        ActivityPlan plan;
        plan.preferred_mode = trip.mode;
        plan.activities.push_back({origin_it->second, t,        0.0, ActivityType::Home});
        plan.activities.push_back({dest_it->second,   t+3600.0, 0.0, ActivityType::Work});
        cold.plans[i] = std::move(plan);
        Event ev{t, static_cast<AgentId>(i),
                 static_cast<uint32_t>(origin_it->second), EventType::AgentDepart, {}};
        eq.push(ev);
    }
    return trips_.size();
}

} // namespace nomad
