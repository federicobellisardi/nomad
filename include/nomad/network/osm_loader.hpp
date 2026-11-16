#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>
#include <nomad/network/intersection.hpp>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── Road class defaults ───────────────────────────────────────────────────────
// Indexed by OSM highway= tag value. Provides speed, capacity, and lane
// count defaults when the way lacks explicit maxspeed= or lanes= tags.
struct RoadClassDefaults {
    RoadClass klass;
    float     default_speed_kmh;
    int       default_lanes;
    float     capacity_per_lane_veh_h;  // veh/h per lane (HCM values)
};

// ── OSM tag configuration ─────────────────────────────────────────────────────
// Loaded from data/schemas/osm_tag_config.yaml.
// Data-driven: new highway types or access rules require only YAML edits.
struct OsmTagConfig {
    // highway= tag → road class defaults
    std::unordered_map<std::string, RoadClassDefaults> highway_map;

    // Mode access control: lists of access= values that grant mode access.
    // e.g., car_access = {"yes", "permissive", "designated", "private"}
    std::vector<std::string> car_access_tags;
    std::vector<std::string> bike_access_tags;
    std::vector<std::string> pedestrian_access_tags;

    // Tags that unconditionally exclude a way from the car network
    std::vector<std::string> car_exclude_tags;

    // Legacy capacity (pre-3112cd0): every direction gets `lanes` (total over
    // both directions, or the class default) x per-lane capacity. Lanes per
    // direction (Graph::edge_lanes, used for LTM storage) are unaffected.
    // Lets storage and capacity effects be tested separately.
    bool capacity_from_total_lanes = false;

    // Default signal timing used when OSM lacks explicit timing data
    SignalDefaults signal_defaults;

    // Green fraction for mid-block signals (opt-in, <= 0 = off): a signal
    // tagged as a pedestrian crossing, or on a node with at most two car
    // neighbours (no crossing road), gets this green share instead of
    // signal_defaults.green_frac. A pedestrian signal rests in green for cars
    // and gives a short call-based phase; the intersection default (0.45)
    // under-states its capacity (Palma: 273 of 374 signals are mid-block).
    float midblock_signal_green_frac = -1.0f;

    static OsmTagConfig load_yaml(const std::filesystem::path& yaml_path);
    static OsmTagConfig defaults();  // built-in defaults for immediate use
};

// ── Raw edge (intermediate representation before graph build) ─────────────────
struct RawEdge {
    OsmWayId  way_id;
    OsmNodeId from_node;
    OsmNodeId to_node;
    float     length_m;
    float     speed_ms;
    float     capacity;
    float     lanes = 1.0f; // lanes in this edge's direction
    bool      along_way = true; // runs in the OSM way's node order
    uint8_t   road_class;
    uint8_t   flags;        // EdgeFlags bitmask
    uint16_t  way_meta_idx;
    std::string name;
    // Intermediate geometry points (excluding endpoints)
    std::vector<std::pair<float, float>> shape;
};

// ── OSM loader ────────────────────────────────────────────────────────────────
// Parses an OSM PBF file in three passes using libosmium:
//   Pass 1 – collect node coordinates into a location index
//   Pass 2 – extract way topology and tags → RawEdge list
//   Pass 3 – extract restriction relations and traffic_signal node metadata
//
// Design notes:
//   - Memory: libosmium streams PBF blocks; only referenced nodes are kept.
//   - Speed: a 100 MB city PBF (e.g., Île-de-France excerpt) loads in ~20s.
//   - Tags: governed by OsmTagConfig; missing tags use class defaults.
//   - Bidirectional edges: each way becomes 2 directed edges (forward + reverse).
//     EdgeId pairs are consecutive (forward=2k, reverse=2k+1).
class OsmLoader {
public:
    explicit OsmLoader(OsmTagConfig cfg = OsmTagConfig::defaults());

    // Parse osm_pbf and return a fully constructed Graph (unsimplified).
    Graph load(const std::filesystem::path& osm_pbf);

    // Convenience: load + clean in one call
    Graph load_and_clean(const std::filesystem::path& osm_pbf,
                          bool simplify_topology = true);

    // Turn restrictions extracted during load (needed by CH preprocessing)
    const std::vector<TurnRestriction>& turn_restrictions() const noexcept {
        return turn_restrictions_;
    }

    // Intersection metadata extracted during load
    const std::unordered_map<OsmNodeId, IntersectionMeta>&
    intersection_meta() const noexcept { return intersections_; }

private:
    Graph build_graph(std::vector<RawEdge>& raw_edges,
                      const std::unordered_map<OsmNodeId, std::pair<float,float>>& coords);

    float infer_speed   (const std::string& maxspeed_tag, RoadClass klass) const;
    float infer_capacity(int lanes, RoadClass klass)                        const;
    bool  car_accessible(const std::string& access_tag,
                          const std::string& highway_tag)                   const;

    OsmTagConfig    cfg_;
    std::vector<TurnRestriction>                     turn_restrictions_;
    std::unordered_map<OsmNodeId, IntersectionMeta>  intersections_;
};

} // namespace nomad
