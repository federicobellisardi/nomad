#pragma once

#include <nomad/core/agent.hpp>
#include <nomad/core/event.hpp>
#include <nomad/core/types.hpp>
#include <nomad/transit/gtfs_loader.hpp>

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── Transit vehicle ───────────────────────────────────────────────────────────
// A scheduled PT vehicle following a TransitLine.
struct TransitVehicle {
    uint32_t    vehicle_id;
    uint32_t    line_idx;          // index into TransitModel::lines_
    uint32_t    current_stop;      // index into TransitLine::stops
    uint32_t    occupancy;
    uint32_t    capacity;
    SimTime     next_departure;
    std::vector<AgentId> on_board;
};

// ── Waiting agent ─────────────────────────────────────────────────────────────
struct WaitingAgent {
    AgentId    agent;
    NodeId     board_stop;
    NodeId     alight_stop;
    SimTime    arrived_at_stop;    // for computing waiting time statistics
};

// ── Transit model ─────────────────────────────────────────────────────────────
// Manages scheduled PT vehicles and agent boarding/alighting.
//
// Agent lifecycle for PT trip:
//   1. AgentDepart → agent walks to nearest stop (AgentEnterLink on walk edges)
//   2. AgentArriveActivity → agent arrives at stop node → WaitingAgent enqueued
//   3. TransitVehicleArrival event → check capacity, board waiting agents
//   4. TransitVehicleDeparture event → vehicle moves to next stop
//   5. Agent alights at destination stop → AgentArriveActivity
//
// Integration with routing:
//   For walk+transit trips: the router returns a Route with:
//     - Walk edges from origin to board_stop
//     - A special PT edge (board_stop → alight_stop)
//     - Walk edges from alight_stop to destination
//   The PT edge has a variable travel time determined by the transit schedule.
//
// Comparison with MATSim's TransitSimulationEngine:
//   MATSim models PT vehicles as agents on the car network (they share roads).
//   nomad models PT vehicles as event-driven entities (separate from road simulation),
//   which is more accurate for rail/metro/tram that don't share road capacity.
//   Buses sharing road capacity can be added via the QueueTrafficModel.
class TransitModel {
public:
    explicit TransitModel(std::vector<TransitLine> lines);

    // Called by simulation engine at startup to inject vehicle departure events
    void schedule_vehicles(EventQueue& eq, SimTime start, SimTime end);

    // Called when an agent reaches a transit stop
    void agent_arrives_at_stop(AgentId a, NodeId stop,
                                NodeId dest_stop, SimTime t);

    // Called when a vehicle arrives at a stop (triggered by vehicle event)
    void vehicle_arrives(uint32_t vehicle_id, SimTime t, EventQueue& eq,
                          AgentHotStore& hot);

    // Query next departure from stop on line toward destination
    std::optional<SimTime> next_departure(NodeId from_stop, NodeId to_stop,
                                           SimTime after_time) const;

    // Expected travel time for PT segment [s]
    float travel_time(NodeId from_stop, NodeId to_stop) const;

    // Current load on vehicle (for Python diagnostics)
    uint32_t vehicle_occupancy(uint32_t vehicle_id) const;

    const std::vector<TransitLine>& lines() const noexcept { return lines_; }

private:
    std::vector<TransitLine>    lines_;
    std::vector<TransitVehicle> vehicles_;

    // Stop → waiting agents
    std::unordered_map<NodeId, std::vector<WaitingAgent>> waiting_;
    mutable std::mutex waiting_mu_;

    // Stop → {line_idx, departure_times}
    std::unordered_map<NodeId,
        std::vector<std::pair<uint32_t, SimTime>>> stop_departures_;
};

} // namespace nomad
