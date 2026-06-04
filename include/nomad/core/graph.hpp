#pragma once

#include <nomad/core/types.hpp>

#include <cassert>
#include <cmath>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace nomad {

// ── Edge data ─────────────────────────────────────────────────────────────────
// 20 bytes — 3 entries fit in a single 64-byte cache line.
// Layout: hot routing fields first (target, length, speed, capacity),
// metadata last. This ensures the Dijkstra inner loop never loads cold data.
//
// Bidirectional edges are stored as consecutive pairs:
//   forward edge = EdgeId 2k,  reverse edge = EdgeId 2k+1
//   reverse_of(e) == e ^ 1
struct EdgeData {
    NodeId   target;           // 4  destination node
    float    length_m;         // 4  pre-computed Haversine length [m]
    float    free_flow_speed;  // 4  m/s (from maxspeed= or class default)
    float    capacity;         // 4  veh/h (lanes × per-lane capacity)
    uint8_t  road_class;       // 1  RoadClass enum value
    uint8_t  flags;            // 1  EdgeFlags bitmask
    uint16_t way_meta_idx;     // 2  index into Graph::way_names / way_ids
};
static_assert(sizeof(EdgeData) == 20, "EdgeData must be exactly 20 bytes");

// ── Node data ─────────────────────────────────────────────────────────────────
// 16 bytes — 4 entries per cache line.
// Coordinates are stored as float (±180° encoded in 32 bits gives ~1cm precision
// at the equator, sufficient for routing).
struct NodeData {
    float    lon;               // 4  WGS84 longitude
    float    lat;               // 4  WGS84 latitude
    uint8_t  intersection_type; // 1  IntersectionType enum value
    uint8_t  signal_phase_idx;  // 1  index into Graph::signal_schedules (0 = none)
    uint16_t _pad;              // 2
    uint32_t osm_node_idx;      // 4  index into Graph::osm_node_ids
};
static_assert(sizeof(NodeData) == 16, "NodeData must be exactly 16 bytes");

// ── Signal schedule ───────────────────────────────────────────────────────────
struct SignalSchedule {
    uint16_t cycle_s;          // total cycle length [s]
    uint8_t  green_pct;        // green fraction [0–100]
    uint8_t  _pad;
};

// ── Graph ─────────────────────────────────────────────────────────────────────
// Compressed Sparse Row (CSR) directed graph.
//
// Access pattern during routing:
//   hot   — row_ptr, col_idx, edges  (Dijkstra kernel reads these)
//   warm  — nodes                    (A* heuristic reads coordinates)
//   cold  — geom_*, osm_*, way_*    (only for output / visualization)
struct Graph {
    // ── HOT: CSR topology ────────────────────────────────────────────────────
    std::vector<uint32_t> row_ptr;   // size: num_nodes + 1
    std::vector<EdgeId>   col_idx;   // size: num_edges, parallel to edges[]
    std::vector<EdgeData> edges;     // size: num_edges

    // ── WARM: node positions ─────────────────────────────────────────────────
    std::vector<NodeData> nodes;

    // ── COLD: geometry sidecar ───────────────────────────────────────────────
    // Each edge may have intermediate shape points beyond just source→target.
    // geom_ptr[e] is the offset into geom_coords for edge e;
    // geom_ptr[e+1] - geom_ptr[e] gives the point count.
    std::vector<uint32_t>                    geom_ptr;    // size: num_edges + 1
    std::vector<std::pair<float, float>>     geom_coords; // (lon, lat) pairs

    // ── COLD: OSM metadata tables ─────────────────────────────────────────────
    std::vector<OsmNodeId>    osm_node_ids;   // indexed by NodeData::osm_node_idx
    std::vector<OsmWayId>     way_ids;        // indexed by EdgeData::way_meta_idx
    std::vector<std::string>  way_names;      // indexed by EdgeData::way_meta_idx
    std::vector<SignalSchedule> signal_schedules; // indexed by NodeData::signal_phase_idx

    // ── Accessors ─────────────────────────────────────────────────────────────
    uint32_t num_nodes() const noexcept {
        return static_cast<uint32_t>(nodes.size());
    }
    uint32_t num_edges() const noexcept {
        return static_cast<uint32_t>(edges.size());
    }

    // Outgoing edges of node u as a span over col_idx
    std::span<const EdgeId> out_edges(NodeId u) const noexcept {
        assert(u < num_nodes());
        return {col_idx.data() + row_ptr[u], col_idx.data() + row_ptr[u + 1]};
    }

    // O(1) reverse edge: forward=2k, reverse=2k+1
    static constexpr EdgeId reverse_of(EdgeId e) noexcept { return e ^ EdgeId{1}; }

    // Free-flow travel time for an edge [seconds]
    float free_flow_time(EdgeId e) const noexcept {
        const auto& ed = edges[e];
        return ed.free_flow_speed > 0.0f ? ed.length_m / ed.free_flow_speed : 1e9f;
    }

    // Haversine distance between two nodes [meters] — used as A* heuristic
    float haversine(NodeId a, NodeId b) const noexcept;

    // Validate internal consistency (used in tests)
    bool validate() const;
};

// ── Haversine inline definition ───────────────────────────────────────────────
inline float Graph::haversine(NodeId a, NodeId b) const noexcept {
    constexpr float R = 6'371'000.0f; // Earth radius [m]
    constexpr float kDeg = 3.14159265f / 180.0f;
    const auto& na = nodes[a];
    const auto& nb = nodes[b];
    float dlat = (nb.lat - na.lat) * kDeg;
    float dlon = (nb.lon - na.lon) * kDeg;
    float slat = std::sin(dlat * 0.5f);
    float slon = std::sin(dlon * 0.5f);
    float alat = std::cos(na.lat * kDeg);
    float blat = std::cos(nb.lat * kDeg);
    float c = slat * slat + alat * blat * slon * slon;
    return 2.0f * R * std::asin(std::sqrt(c));
}

} // namespace nomad
