#pragma once

#include <nomad/core/types.hpp>

#include <cstdint>
#include <vector>

namespace nomad {

// ── Activity plan (cold AoS) ──────────────────────────────────────────────────
// Accessed once per activity transition; stored separately from hot fields.
struct Activity {
    NodeId       location;
    SimTime      start_time;
    SimTime      duration;
    ActivityType type;
};

struct ActivityPlan {
    std::vector<Activity> activities;
    AgentMode             preferred_mode = AgentMode::Car;
};

// ── Hot SoA store (one entry per agent, cache-friendly for bulk ops) ──────────
// These fields are read/written during every event. Keeping them in separate
// arrays means that e.g. scanning all current_edge values for "agents on link X"
// is a contiguous memory read with no stride.
struct AgentHotStore {
    std::vector<EdgeId>     current_edge;
    std::vector<SimTime>    enter_time;
    std::vector<SimTime>    scheduled_exit;
    std::vector<AgentMode>  mode;
    std::vector<AgentState> state;
    std::vector<uint16_t>   route_pos;        // position within compressed route
    std::vector<float>      freeflow_route_s; // pre-computed free-flow route time [s]
    std::vector<uint8_t>    reroute_count;    // number of times this agent has been rerouted

    std::size_t size() const noexcept { return current_edge.size(); }

    void push_back(EdgeId edge, SimTime enter, SimTime sched_exit,
                   AgentMode m, AgentState s, uint16_t rpos) {
        current_edge      .push_back(edge);
        enter_time        .push_back(enter);
        scheduled_exit    .push_back(sched_exit);
        mode              .push_back(m);
        state             .push_back(s);
        route_pos         .push_back(rpos);
        freeflow_route_s  .push_back(0.0f);
        reroute_count     .push_back(0);
    }

    void resize(std::size_t n) {
        current_edge      .resize(n, kInvalidEdge);
        enter_time        .resize(n, kInvalidTime);
        scheduled_exit    .resize(n, kInvalidTime);
        mode              .resize(n, AgentMode::Car);
        state             .resize(n, AgentState::Waiting);
        route_pos         .resize(n, 0);
        freeflow_route_s  .resize(n, 0.0f);
        reroute_count     .resize(n, 0);
    }
};

// ── Cold AoS store ────────────────────────────────────────────────────────────
struct AgentColdStore {
    std::vector<ActivityPlan> plans;

    std::size_t size() const noexcept { return plans.size(); }
    void resize(std::size_t n) { plans.resize(n); }
};

// ── Per-agent route store ─────────────────────────────────────────────────────
// Each agent owns its route as a plain EdgeId vector.  replace_suffix() writes
// in-place into the existing vector (capacity is reused), so repeated rerouting
// never grows total memory — unlike a flat delta-encoded buffer where every
// push_route() appended without freeing the previous allocation.
struct RouteStore {
    std::vector<std::vector<EdgeId>> routes;

    std::size_t size() const noexcept { return routes.size(); }

    void push_route(AgentId agent, const std::vector<EdgeId>& route) {
        if (agent >= routes.size()) routes.resize(agent + 1);
        routes[agent] = route;
    }

    std::vector<EdgeId> get_route(AgentId agent) const {
        if (agent >= routes.size()) return {};
        return routes[agent];
    }

    void replace_suffix(AgentId agent, uint16_t from_pos,
                         const std::vector<EdgeId>& new_suffix) {
        if (agent >= routes.size()) routes.resize(agent + 1);
        auto& r = routes[agent];
        r.resize(from_pos);
        r.insert(r.end(), new_suffix.begin(), new_suffix.end());
    }

    void clear()            { routes.clear(); }
    void resize(std::size_t n) { routes.resize(n); }
};

} // namespace nomad
