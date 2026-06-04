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
    std::vector<uint16_t>   route_pos;     // position within compressed route

    std::size_t size() const noexcept { return current_edge.size(); }

    void push_back(EdgeId edge, SimTime enter, SimTime sched_exit,
                   AgentMode m, AgentState s, uint16_t rpos) {
        current_edge   .push_back(edge);
        enter_time     .push_back(enter);
        scheduled_exit .push_back(sched_exit);
        mode           .push_back(m);
        state          .push_back(s);
        route_pos      .push_back(rpos);
    }

    void resize(std::size_t n) {
        current_edge   .resize(n, kInvalidEdge);
        enter_time     .resize(n, kInvalidTime);
        scheduled_exit .resize(n, kInvalidTime);
        mode           .resize(n, AgentMode::Car);
        state          .resize(n, AgentState::Waiting);
        route_pos      .resize(n, 0);
    }
};

// ── Cold AoS store ────────────────────────────────────────────────────────────
struct AgentColdStore {
    std::vector<ActivityPlan> plans;

    std::size_t size() const noexcept { return plans.size(); }
    void resize(std::size_t n) { plans.resize(n); }
};

// ── Compressed route store ────────────────────────────────────────────────────
// Routes are stored as delta-encoded 16-bit sequences to save memory.
// For 1M agents with avg 40 edges/route: 40M × 2 bytes ≈ 80 MB,
// vs 40M × 4 bytes = 160 MB for plain EdgeId vectors.
//
// Encoding: route for agent i starts at flat_data[offsets[i]].
//   flat_data[offsets[i]] = length (number of edges)
//   flat_data[offsets[i]+1 ... +length] = delta-encoded EdgeId differences
//   base_edges[i] = absolute EdgeId of first edge in route
//
// Decoding: edge[k] = base_edges[i] + sum(flat_data[offsets[i]+1 ... +k+1])
struct RouteStore {
    std::vector<uint32_t> offsets;        // per-agent offset into flat_data
    std::vector<uint16_t> flat_data;      // delta-encoded sequences
    std::vector<EdgeId>   base_edges;     // absolute first edge per agent

    std::size_t size() const noexcept { return offsets.size(); }

    void push_route(AgentId agent, const std::vector<EdgeId>& route);
    std::vector<EdgeId> get_route(AgentId agent) const;

    // Update remaining route from route_pos onward (called on re-routing)
    void replace_suffix(AgentId agent, uint16_t from_pos,
                         const std::vector<EdgeId>& new_suffix);

    void clear() {
        offsets.clear();
        flat_data.clear();
        base_edges.clear();
    }

    void resize(std::size_t n) {
        offsets    .resize(n, 0);
        base_edges .resize(n, kInvalidEdge);
    }
};

} // namespace nomad
