#include <nomad/demand/synthetic.hpp>
#include <nomad/demand/departure_sampler.hpp>
#include <cmath>
#include <spdlog/spdlog.h>

namespace nomad {

GravityDemand::GravityDemand() : GravityDemand(Config{}) {}
GravityDemand::GravityDemand(Config cfg) : cfg_(cfg), rng_(cfg.seed) {}

std::vector<std::pair<NodeId, NodeId>>
GravityDemand::build_trip_table(const Graph& g) {
    std::vector<std::pair<NodeId, NodeId>> trips;
    trips.reserve(cfg_.total_agents);
    const uint32_t N = g.num_nodes();
    if (N < 2) return trips;

    std::uniform_int_distribution<uint32_t> node_dist(0, N - 1);
    for (uint32_t k = 0; k < cfg_.total_agents; ++k) {
        NodeId o = node_dist(rng_);
        NodeId d = node_dist(rng_);
        float dist_m = g.haversine(o, d);
        float weight = std::exp(-cfg_.beta * dist_m);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        trips.push_back(u(rng_) < weight ? std::make_pair(o, d)
                                          : std::make_pair(o, node_dist(rng_)));
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
        // Event field order: time, agent, payload(NodeId), type, pad
        Event ev{t, static_cast<AgentId>(i),
                 static_cast<uint32_t>(trips[i].first), EventType::AgentDepart, {}};
        eq.push(ev);
    }
    spdlog::info("GravityDemand: generated {} agents", trips.size());
    return trips.size();
}

RadiationDemand::RadiationDemand() : RadiationDemand(Config{}) {}
RadiationDemand::RadiationDemand(Config cfg) : cfg_(cfg), rng_(cfg.seed) {}

std::size_t RadiationDemand::generate(const Graph& graph, EventQueue& eq,
                                        AgentColdStore& cold, RouteStore& routes) {
    const uint32_t N = graph.num_nodes();
    cold.resize(cfg_.total_agents);
    routes.resize(cfg_.total_agents);
    std::uniform_int_distribution<uint32_t> nd(0, N > 0 ? N - 1 : 0);
    DepartureSampler sampler = DepartureSampler::lognormal(cfg_.peak_mean_s, cfg_.peak_std_s);
    for (uint32_t i = 0; i < cfg_.total_agents; ++i) {
        NodeId o = nd(rng_), d = nd(rng_);
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
