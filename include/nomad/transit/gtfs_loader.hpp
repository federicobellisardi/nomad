#pragma once

#include <nomad/core/types.hpp>
#include <nomad/network/multilayer_graph.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── GTFS data structures ──────────────────────────────────────────────────────
// Follows the GTFS specification (https://gtfs.org/reference/static).
// nomad reads: stops.txt, routes.txt, trips.txt, stop_times.txt, calendar.txt.
// Optional: shapes.txt (for geometry), transfers.txt (for explicit transfers).
//
// Integration with OSM:
//   GTFS stops are matched to OSM nodes by:
//   1. Exact OSM node match (stop_id present as OSM ref)
//   2. Nearest OSM node within snap_radius_m (default 50m)
//   3. New node insertion if no match found within snap_radius_m
//
// The resulting transit network becomes the TransitModel layer in MultiLayerGraph.

struct GtfsStop {
    std::string stop_id;
    std::string stop_name;
    float       lon, lat;
    NodeId      road_node;     // mapped to road graph via snap
    std::string zone_id;
    uint8_t     wheelchair_boarding;
};

struct GtfsRoute {
    std::string route_id;
    std::string route_short_name;
    std::string route_long_name;
    uint8_t     route_type;   // GTFS type: 0=tram, 1=metro, 2=rail, 3=bus, 4=ferry
    std::string agency_id;
};

struct GtfsStopTime {
    std::string stop_id;
    SimTime     arrival_s;    // seconds since midnight
    SimTime     departure_s;
    uint16_t    stop_sequence;
};

struct GtfsTrip {
    std::string trip_id;
    std::string route_id;
    std::string service_id;
    std::string direction_id;
    std::vector<GtfsStopTime> stop_times;
};

struct GtfsServiceCalendar {
    std::string service_id;
    bool        monday, tuesday, wednesday, thursday, friday, saturday, sunday;
    std::string start_date, end_date;
};

// ── Transit line ──────────────────────────────────────────────────────────────
// A processed GTFS route+direction with ordered stop sequences and
// headway-based or exact departure times.
struct TransitLine {
    std::string     route_id;
    std::string     route_name;
    TransportLayer  mode;          // Bus/Tram/Metro/Rail/Ferry
    std::vector<NodeId> stops;     // ordered road graph nodes
    std::vector<float>  dwell_s;   // dwell time at each stop [s]
    std::vector<SimTime> departures; // sorted list of departure times from stop[0]
    uint32_t        capacity;      // max passengers per vehicle
};

// ── GTFS loader ───────────────────────────────────────────────────────────────
// Reads a GTFS feed directory (unzipped) and builds TransitLine objects.
// The TransitLine list is then consumed by TransitModel to generate
// PT vehicle events in the simulation engine.
//
// Comparison with alternatives:
//   OpenTripPlanner: uses RAPTOR for optimal multi-leg transit routing.
//     Pros: exact optimal routing. Cons: Java, not embeddable in C++ sim.
//   R5: similar to OTP, faster. Still Java/Kotlin.
//   SUMO: own GTFS importer → pt2flows. Generates vehicle flows.
//     Pros: integrated with SUMO's signal model. Cons: SUMO-specific.
//   MATSim: TransitScheduleReader. Reads PT as separate network layer.
//     Pros: well-tested. Cons: XML-heavy, Java.
//
// nomad approach: C++ GTFS reader, generates TransitLine objects that
// drive event-based vehicle scheduling. Agents can board/alight via
// standard AgentEnterLink/ExitLink events on special PT edges.
// For optimal multi-modal routing, RAPTOR is planned as a Phase 6 addition.
// Config defined outside GtfsLoader to avoid GCC 13 bug with nested-struct
// default member initializers used as constructor default arguments.
struct GtfsLoaderConfig {
    float snap_radius_m  = 50.0f;
    bool  insert_nodes   = true;
    bool  load_shapes    = false;
    std::string service_date = "";
};

class GtfsLoader {
public:
    using Config = GtfsLoaderConfig;

    explicit GtfsLoader(Config cfg = {});

    // Load GTFS feed from directory and snap stops to road graph
    std::vector<TransitLine> load(const std::filesystem::path& gtfs_dir,
                                   MultiLayerGraph& mlg);

    // All stops extracted (before snapping)
    const std::vector<GtfsStop>& stops() const noexcept { return stops_; }

    // Statistics
    struct LoadStats {
        uint32_t stops_snapped;
        uint32_t stops_inserted;
        uint32_t stops_dropped;
        uint32_t routes_loaded;
        uint32_t trips_loaded;
    };
    const LoadStats& stats() const noexcept { return stats_; }

private:
    void load_stops(const std::filesystem::path& dir);
    void load_routes(const std::filesystem::path& dir);
    void load_trips(const std::filesystem::path& dir);
    void load_stop_times(const std::filesystem::path& dir);

    void snap_stops_to_graph(MultiLayerGraph& mlg);
    std::vector<TransitLine> build_lines() const;

    static TransportLayer route_type_to_layer(uint8_t rt);

    Config  cfg_;
    std::vector<GtfsStop>            stops_;
    std::unordered_map<std::string, GtfsRoute> routes_;
    std::unordered_map<std::string, GtfsTrip>  trips_;
    LoadStats stats_{};
};

} // namespace nomad
