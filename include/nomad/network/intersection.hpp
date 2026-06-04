#pragma once

#include <nomad/core/types.hpp>

#include <cstdint>
#include <vector>

namespace nomad {

// ── Turn restriction ──────────────────────────────────────────────────────────
// Sourced from OSM relation type=restriction.
// Enforced during CH preprocessing (restricted turns become infinite-cost
// edges), so there is zero runtime cost during simulation.
struct TurnRestriction {
    OsmWayId  from_way;
    OsmWayId  to_way;
    OsmNodeId via_node;
    enum class Type : uint8_t { NoTurn = 0, OnlyTurn = 1 } type;
};

// ── Signal phase ──────────────────────────────────────────────────────────────
// Simplified fixed-time signal plan. Where OSM provides cycle data it is used;
// otherwise city-specific defaults apply (90s cycle, 45% green fraction).
// Adaptive signal control is exposed via the Python hook system (Phase 5).
struct SignalPhase {
    uint8_t phase_id;
    float   green_s;    // green duration [s]
    float   cycle_s;    // total cycle length [s]
    // Affected movements encoded as bitmask (N=0,NE=1,...,NW=7 → S=8..SW=15)
    uint16_t movement_mask;
};

// ── Intersection metadata ─────────────────────────────────────────────────────
struct IntersectionMeta {
    OsmNodeId             osm_id;
    IntersectionType      type;
    std::vector<SignalPhase>      phases;  // empty for unsignalized
    std::vector<TurnRestriction>  turns;   // from OSM restriction relations
};

// ── Capacity derate factors (from HCM / TCQSM) ────────────────────────────────
// Applied when building the graph to adjust EdgeData::capacity.
// Signalized:   effective_cap = base_cap × (green_s / cycle_s) × kSignalSaturation
// Stop sign:    effective_cap = base_cap × kStopSignDerate (minor road only)
// Roundabout:   effective_cap = base_cap × kRoundaboutDerate
namespace CapacityDerate {
    inline constexpr float kSignalSaturation = 0.90f; // lost time factor (HCM)
    inline constexpr float kStopSignDerate   = 0.60f; // minor road empirical (TCQSM)
    inline constexpr float kRoundaboutDerate = 0.75f; // entry flow constraint
}

// ── Default signal timing by region ──────────────────────────────────────────
struct SignalDefaults {
    float cycle_s       = 90.0f;
    float green_frac    = 0.45f;
};

// Compute the effective capacity for an outgoing edge, given the intersection
// type and signal schedule at the edge's source node.
float effective_capacity(float base_capacity,
                          IntersectionType type,
                          const SignalPhase* phase = nullptr) noexcept;

} // namespace nomad
