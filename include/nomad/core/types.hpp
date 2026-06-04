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
// Seconds since midnight. double gives sub-millisecond precision over 24h.
using SimTime = double;
inline constexpr SimTime kInvalidTime = -1.0;

// ── Road classification ───────────────────────────────────────────────────────
// Ordered coarsely from fastest (motorway) to slowest (footway).
// The numeric value is used as a tiebreaker when two paths have equal cost.
enum class RoadClass : uint8_t {
    Motorway    = 0,
    MotorwayLink= 1,
    Trunk       = 2,
    TrunkLink   = 3,
    Primary     = 4,
    PrimaryLink = 5,
    Secondary   = 6,
    SecondaryLink=7,
    Tertiary    = 8,
    TertiaryLink= 9,
    Residential = 10,
    LivingStreet= 11,
    Service     = 12,
    Unclassified= 13,
    Track       = 14,
    Cycleway    = 15,
    Footway     = 16,
    Path        = 17,
    Steps       = 18,
    Unknown     = 255,
};

// ── Intersection type ─────────────────────────────────────────────────────────
enum class IntersectionType : uint8_t {
    Simple          = 0,   // no explicit control
    StopSign        = 1,
    AllWayStop      = 2,
    TrafficSignal   = 3,
    Roundabout      = 4,
    MiniRoundabout  = 5,
    MotorwayJunction= 6,
};

// ── Agent mode (transport mode) ───────────────────────────────────────────────
enum class AgentMode : uint8_t {
    Car     = 0,
    Transit = 1,
    Bike    = 2,
    Walk    = 3,
    Idle    = 4,
};

// ── Agent lifecycle state ─────────────────────────────────────────────────────
enum class AgentState : uint8_t {
    Waiting       = 0,   // pre-departure
    OnLink        = 1,   // currently traversing a link
    AtActivity    = 2,   // dwelling at an activity location
    Teleported    = 3,   // moved without network traversal (missing data)
    Arrived       = 4,   // completed all activities
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

// ── Edge flags (bitmask stored in EdgeData::flags) ────────────────────────────
namespace EdgeFlags {
    inline constexpr uint8_t OneWay          = 0x01;
    inline constexpr uint8_t HasSignal       = 0x02;
    inline constexpr uint8_t TurnRestricted  = 0x04;
    inline constexpr uint8_t Bridge          = 0x08;
    inline constexpr uint8_t Tunnel          = 0x10;
    inline constexpr uint8_t Roundabout      = 0x20;
    inline constexpr uint8_t Reversed        = 0x40;  // this is the reverse edge of a pair
}

} // namespace nomad
