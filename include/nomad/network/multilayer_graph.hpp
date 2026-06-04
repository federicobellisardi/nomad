#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── Transport layer identifiers ───────────────────────────────────────────────
// Each transport mode has its own subgraph layer. Agents use layer-specific
// routing. Transfer edges connect layers at intermodal nodes (e.g., park-and-ride,
// transit stops, bike stations).
//
// Design rationale vs. alternatives:
//   Single graph with mode mask (MATSim style): simple, but routing across layers
//   requires careful edge filtering. Preferred for cities where modes share
//   infrastructure (e.g., shared bus/car lanes).
//
//   Separate graphs per mode (A/B Street style): clean separation, allows
//   independent simplification per mode. Required for transit (fundamentally
//   different: schedule-based, not continuous). Preferred for walking/cycling
//   where the network topology differs significantly from car network.
//
//   Time-expanded graph (OTP/R5 style): exact for schedule-based transit but
//   memory-intensive (O(stops × time_slots)). Suitable for static transit
//   planning, not for dynamic agent simulation.
//
// nomad uses a HYBRID approach:
//   - Road layers (car, bike, walk): single CSR graph with mode_mask per edge
//   - Transit layer: schedule-based, separate from the routing graph
//   - Transfer edges: explicit connections between layers (walk to stop, etc.)
//
// This matches the MATSim multi-modal approach and is compatible with ActivitySim.

enum class TransportLayer : uint8_t {
    Car        = 0,
    Bike       = 1,
    Walk       = 2,
    Bus        = 3,
    Tram       = 4,
    Metro      = 5,
    Rail       = 6,
    Ferry      = 7,
    Transfer   = 8,   // intermodal transfer edge (virtual)
    Any        = 0xFF,
};

// Mode bitmask for EdgeData::flags extension
namespace ModeMask {
    inline constexpr uint8_t Car    = 0x01;
    inline constexpr uint8_t Bike   = 0x02;
    inline constexpr uint8_t Walk   = 0x04;
    inline constexpr uint8_t PT     = 0x08;  // any public transport
}

// ── Transfer node ─────────────────────────────────────────────────────────────
// A point in space where mode changes can occur.
// Examples: bus stop, metro station, park-and-ride lot, bike station.
struct TransferNode {
    NodeId     road_node;        // nearest car/walk graph node
    OsmNodeId  osm_id;
    float      lon, lat;
    float      walk_access_s;    // expected walking time to nearest stop [s]
    uint8_t    available_modes;  // ModeMask bitmask of accessible modes
    std::string name;
};

// ── Transfer edge ─────────────────────────────────────────────────────────────
// Connects a node in one mode layer to a node in another.
// Cost includes: walking time, waiting time, transfer penalty.
struct TransferEdge {
    NodeId         from_node;
    NodeId         to_node;
    TransportLayer from_layer;
    TransportLayer to_layer;
    float          walk_time_s;     // walking leg [s]
    float          penalty_s;       // generalised cost penalty [s equivalent]
};

// ── Multi-layer graph ─────────────────────────────────────────────────────────
// Wraps a single road Graph (with per-edge mode masks) and a set of transfer
// nodes + edges. The transit schedule is held separately in TransitModel.
//
// Routing approach per layer:
//   Car / Bike / Walk: A* or CH on the road graph, filtered by mode_mask
//   Transit: Raptor algorithm on the GTFS schedule (see transit/gtfs_loader.hpp)
//   Multimodal: profile search or time-expanded query composing layers
struct MultiLayerGraph {
    std::shared_ptr<Graph> road;          // primary road network (all modes)
    std::vector<TransferNode> transfers;  // intermodal nodes
    std::vector<TransferEdge> xfer_edges; // intermodal connections

    // Spatial lookup: find transfer nodes within radius_m of (lon, lat)
    std::vector<const TransferNode*>
    find_transfers_near(float lon, float lat, float radius_m) const;

    // Mode-filtered edge cost: returns free-flow time if mode is allowed, else inf
    float edge_cost_for_mode(EdgeId e, AgentMode mode) const noexcept;

    // Check if a node is accessible for a given mode
    bool node_accessible(NodeId n, AgentMode mode) const noexcept;

    // Build mode-filtered subgraph view (lazy, cached)
    // Returns a view of edges accessible to the given mode.
    std::span<const EdgeId> mode_edges(NodeId u, AgentMode mode) const;
};

// ── Mode filter ───────────────────────────────────────────────────────────────
// Maps AgentMode → acceptable road classes and edge flags.
struct ModeFilter {
    AgentMode mode;
    uint8_t   required_mask;     // must have ALL bits
    uint8_t   forbidden_mask;    // must have NONE of these bits
    float     max_road_class;    // max RoadClass value (higher = slower roads OK)

    static ModeFilter for_mode(AgentMode m);
};

} // namespace nomad
