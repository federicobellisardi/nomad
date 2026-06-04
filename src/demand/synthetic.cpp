#include <nomad/demand/synthetic.hpp>
#include <nomad/demand/departure_sampler.hpp>
#include <cmath>
#include <spdlog/spdlog.h>

namespace nomad {

GravityDemand::GravityDemand() : GravityDemand(Config{}) {}
GravityDemand::GravityDemand(Config cfg) : cfg_(cfg), rng_(cfg.seed) {}

// ── Build list of nodes accessible by cfg_.mode ────────────────────────────────
// Avoids routing agents through footway/steps/cycleway when mode=car.
static std::vector<NodeId> accessible_nodes(const Graph& g, AgentMode mode) {
    std::vector<NodeId> result;
    result.reserve(g.num_nodes() / 2);
    for (uint32_t u = 0; u < g.num_nodes(); ++u) {
        for (EdgeId eid : g.out_edges(u)) {
            auto klass = static_cast<RoadClass>(g.edges[eid].road_class);
            if (road_class_accessible(klass, mode)) {
                result.push_back(u);
                break; // at least one accessible edge → node is usable
            }
        }
    }
    if (result.empty()) {
        // Fallback: use all nodes (should never happen for a valid city graph)
        spdlog::warn("No mode-accessible nodes found for mode {}; using all nodes",
                     static_cast<int>(mode));
        for (uint32_t u = 0; u < g.num_nodes(); ++u) result.push_back(u);
    }
    spdlog::info("Mode-accessible nodes: {}/{}", result.size(), g.num_nodes());
    return result;
}

std::vector<std::pair<NodeId, NodeId>>
GravityDemand::build_trip_table(const Graph& g) {
    std::vector<std::pair<NodeId, NodeId>> trips;
    trips.reserve(cfg_.total_agents);

    // Filter to mode-accessible nodes only
    const auto nodes = accessible_nodes(g, cfg_.mode);
    if (nodes.size() < 2) return trips;

    std::uniform_int_distribution<uint32_t> nd(0, static_cast<uint32_t>(nodes.size() - 1));

    for (uint32_t k = 0; k < cfg_.total_agents; ++k) {
        NodeId o = nodes[nd(rng_)];
        NodeId d = nodes[nd(rng_)];
        float dist_m = g.haversine(o, d);
        float weight = std::exp(-cfg_.beta * dist_m);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        // Gravity: accept (o,d) with probability ∝ exp(-beta*dist), else pick random d
        trips.push_back(u(rng_) < weight
            ? std::make_pair(o, d)
            : std::make_pair(o, nodes[nd(rng_)]));
    }
    return trips;
}

std::size_t GravityDemand::generate(const Graph& graph, EventQueue& eq,
                                      AgentColdStore& cold, RouteStore& routes) {
    auto trips = build_trip_table(graph);
    cold.resize(trips.size());
    routes.resize(trips.size());

    DepartureSampler sampler = DepartureSampler::lognormal(cfg_.peak_mean_s, cfg_.peak_std_s);
    for (std::size_t i = 0; i < trips.size(); ++i) {
        SimTime t = sampler.sample(rng_);
        ActivityPlan plan;
        plan.preferred_mode = cfg_.mode;
        plan.activities.push_back({trips[i].first,  t,        0.0, ActivityType::Home});
        plan.activities.push_back({trips[i].second, t+3600.0, 0.0, ActivityType::Work});
        cold.plans[i] = std::move(plan);
        Event ev{t, static_cast<AgentId>(i),
                 static_cast<uint32_t>(trips[i].first), EventType::AgentDepart, {}};
        eq.push(ev);
    }
    spdlog::info("GravityDemand: generated {} agents (mode={})",
                  trips.size(), static_cast<int>(cfg_.mode));
    return trips.size();
}

RadiationDemand::RadiationDemand() : RadiationDemand(Config{}) {}
RadiationDemand::RadiationDemand(Config cfg) : cfg_(cfg), rng_(cfg.seed) {}

std::size_t RadiationDemand::generate(const Graph& graph, EventQueue& eq,
                                        AgentColdStore& cold, RouteStore& routes) {
    const auto nodes = accessible_nodes(graph, cfg_.mode);
    cold.resize(cfg_.total_agents);
    routes.resize(cfg_.total_agents);
    std::uniform_int_distribution<uint32_t> nd(0, static_cast<uint32_t>(nodes.size() - 1));
    DepartureSampler sampler = DepartureSampler::lognormal(cfg_.peak_mean_s, cfg_.peak_std_s);
    for (uint32_t i = 0; i < cfg_.total_agents; ++i) {
        NodeId o = nodes[nd(rng_)];
        NodeId d = nodes[nd(rng_)];
        SimTime t = sampler.sample(rng_);
        ActivityPlan plan;
        plan.preferred_mode = cfg_.mode;
        plan.activities.push_back({o, t,        0.0, ActivityType::Home});
        plan.activities.push_back({d, t+3600.0, 0.0, ActivityType::Work});
        cold.plans[i] = std::move(plan);
        Event ev{t, static_cast<AgentId>(i),
                 static_cast<uint32_t>(o), EventType::AgentDepart, {}};
        eq.push(ev);
    }
    return cfg_.total_agents;
}

} // namespace nomad
