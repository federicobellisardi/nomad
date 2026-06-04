#pragma once

#include <cstdint>
#include <limits>

namespace nomad {

// ── Identifier types ──────────────────────────────────────────────────────────
using NodeId   = uint32_t;
using EdgeId   = uint32_t;
using AgentId  = uint32_t;
using ZoneId   = uint32_t;
using OsmNodeId = int64_t;
using OsmWayId  = int64_t;
using OsmRelId  = int64_t;

inline constexpr NodeId  kInvalidNode  = std::numeric_limits<NodeId>::max();
inline constexpr EdgeId  kInvalidEdge  = std::numeric_limits<EdgeId>::max();
inline constexpr AgentId kNoAgent      = std::numeric_limits<AgentId>::max();
inline constexpr ZoneId  kInvalidZone  = std::numeric_limits<ZoneId>::max();

// ── Time ──────────────────────────────────────────────────────────────────────
using SimTime = double;
inline constexpr SimTime kInvalidTime = -1.0;

// ── Road classification ───────────────────────────────────────────────────────
// Ordered from fastest (motorway) to slowest (steps).
enum class RoadClass : uint8_t {
    Motorway     = 0,
    MotorwayLink = 1,
    Trunk        = 2,
    TrunkLink    = 3,
    Primary      = 4,
    PrimaryLink  = 5,
    Secondary    = 6,
    SecondaryLink= 7,
    Tertiary     = 8,
    TertiaryLink = 9,
    Residential  = 10,
    LivingStreet = 11,
    Service      = 12,
    Unclassified = 13,
    Track        = 14,
    Cycleway     = 15,  // ← cars cannot use from here on
    Footway      = 16,
    Path         = 17,
    Steps        = 18,
    Unknown      = 255,
};

// ── Agent mode (transport mode) ───────────────────────────────────────────────
// Declared before RoadClass helpers so road_class_accessible can reference it.
enum class AgentMode : uint8_t {
    Car     = 0,
    Transit = 1,
    Bike    = 2,
    Walk    = 3,
    Idle    = 4,
};

// ── Mode accessibility ────────────────────────────────────────────────────────
// Returns true if agents of the given mode may traverse an edge of this class.
// Used by A* routing (edge filtering) and demand generation (node filtering).
inline constexpr bool road_class_accessible(RoadClass klass, AgentMode mode) noexcept {
    switch (mode) {
    case AgentMode::Car:
        return klass <= RoadClass::Track;          // 0–14 only
    case AgentMode::Bike:
        return klass != RoadClass::Motorway &&
               klass != RoadClass::MotorwayLink &&
               klass != RoadClass::Steps;
    case AgentMode::Walk:
        return klass != RoadClass::Motorway &&
               klass != RoadClass::MotorwayLink;
    case AgentMode::Transit:
        return true;                               // transit graph: Phase 6
    default:
        return true;
    }
}

// ── Intersection type ─────────────────────────────────────────────────────────
enum class IntersectionType : uint8_t {
    Simple           = 0,
    StopSign         = 1,
    AllWayStop       = 2,
    TrafficSignal    = 3,
    Roundabout       = 4,
    MiniRoundabout   = 5,
    MotorwayJunction = 6,
};

// ── Agent lifecycle state ─────────────────────────────────────────────────────
enum class AgentState : uint8_t {
    Waiting       = 0,
    OnLink        = 1,
    AtActivity    = 2,
    Teleported    = 3,
    Arrived       = 4,
};

// ── Event types ───────────────────────────────────────────────────────────────
enum class EventType : uint8_t {
    AgentDepart         = 0,
    AgentEnterLink      = 1,
    AgentExitLink       = 2,
    AgentArriveActivity = 3,
    AgentReroute        = 4,
    SignalPhaseChange   = 5,
    DemandInjection     = 6,
    SnapshotDump        = 7,
};

// ── Activity type ─────────────────────────────────────────────────────────────
enum class ActivityType : uint8_t {
    Home    = 0,
    Work    = 1,
    Shop    = 2,
    Leisure = 3,
    School  = 4,
    Other   = 5,
};

// ── Edge flags ────────────────────────────────────────────────────────────────
namespace EdgeFlags {
    inline constexpr uint8_t OneWay         = 0x01;
    inline constexpr uint8_t HasSignal      = 0x02;
    inline constexpr uint8_t TurnRestricted = 0x04;
    inline constexpr uint8_t Bridge         = 0x08;
    inline constexpr uint8_t Tunnel         = 0x10;
    inline constexpr uint8_t Roundabout     = 0x20;
    inline constexpr uint8_t Reversed       = 0x40;
}

} // namespace nomad
