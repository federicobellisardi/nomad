#include <nomad/network/osm_loader.hpp>
#include <nomad/network/intersection.hpp>
#include <nomad/network/network_cleaner.hpp>

// libosmium includes (PBF only — no expat dependency)
#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

namespace nomad {

// ── Default tag configuration ─────────────────────────────────────────────────
OsmTagConfig OsmTagConfig::defaults() {
    OsmTagConfig cfg;

    // highway= → (class, speed_kmh, lanes, capacity_per_lane)
    cfg.highway_map = {
        {"motorway",      {RoadClass::Motorway,     120.0f, 2, 2000.0f}},
        {"motorway_link", {RoadClass::MotorwayLink,  80.0f, 1, 1500.0f}},
        {"trunk",         {RoadClass::Trunk,         100.0f, 2, 1800.0f}},
        {"trunk_link",    {RoadClass::TrunkLink,      60.0f, 1, 1400.0f}},
        {"primary",       {RoadClass::Primary,        80.0f, 2, 1600.0f}},
        {"primary_link",  {RoadClass::PrimaryLink,    50.0f, 1, 1200.0f}},
        {"secondary",     {RoadClass::Secondary,      60.0f, 1, 1000.0f}},
        {"secondary_link",{RoadClass::SecondaryLink,  40.0f, 1,  900.0f}},
        {"tertiary",      {RoadClass::Tertiary,       50.0f, 1,  800.0f}},
        {"tertiary_link", {RoadClass::TertiaryLink,   30.0f, 1,  700.0f}},
        {"residential",   {RoadClass::Residential,    30.0f, 1,  600.0f}},
        {"living_street", {RoadClass::LivingStreet,   10.0f, 1,  200.0f}},
        {"service",       {RoadClass::Service,        30.0f, 1,  600.0f}},
        {"unclassified",  {RoadClass::Unclassified,   30.0f, 1,  500.0f}},
        {"track",         {RoadClass::Track,          20.0f, 1,  400.0f}},
        {"cycleway",      {RoadClass::Cycleway,       15.0f, 1,  300.0f}},
        {"footway",       {RoadClass::Footway,         5.0f, 1,  600.0f}},
        {"path",          {RoadClass::Path,            5.0f, 1,  400.0f}},
        {"steps",         {RoadClass::Steps,           2.0f, 1,  200.0f}},
    };

    // "private" excluded: private roads (parking lots, resort driveways, gated
    // estates) are not part of the public routing network. Including them causes
    // CH to route through low-capacity links, creating artificial bottlenecks.
    cfg.car_access_tags = {"yes", "permissive", "designated",
                            "destination", "delivery", "customers"};
    cfg.bike_access_tags = {"yes", "permissive", "designated", "private"};
    cfg.pedestrian_access_tags = {"yes", "permissive", "designated"};
    cfg.car_exclude_tags = {"no", "emergency", "military"};

    cfg.signal_defaults = {90.0f, 0.45f};
    return cfg;
}

OsmTagConfig OsmTagConfig::load_yaml(const std::filesystem::path& yaml_path) {
    OsmTagConfig cfg = defaults();
    try {
        YAML::Node root = YAML::LoadFile(yaml_path.string());
        if (root["signal_cycle_s"])
            cfg.signal_defaults.cycle_s = root["signal_cycle_s"].as<float>();
        if (root["signal_green_frac"])
            cfg.signal_defaults.green_frac = root["signal_green_frac"].as<float>();
        // Additional tag overrides loaded here as the config file grows
    } catch (const std::exception& ex) {
        spdlog::warn("OsmTagConfig::load_yaml: {} — using defaults", ex.what());
    }
    return cfg;
}

// ── Speed parsing ─────────────────────────────────────────────────────────────
static float parse_maxspeed(std::string_view tag) {
    if (tag.empty()) return -1.0f;
    // Handle "XX mph", "XX km/h", implicit km/h
    float value = -1.0f;
    auto result = std::from_chars(tag.data(), tag.data() + tag.size(), value);
    if (result.ec != std::errc()) return -1.0f;
    std::string_view remainder(result.ptr);
    if (remainder.find("mph") != std::string_view::npos) value *= 1.60934f;
    return value;
}

float OsmLoader::infer_speed(const std::string& maxspeed_tag, RoadClass klass) const {
    float kmh = parse_maxspeed(maxspeed_tag);
    if (kmh > 0.0f) return kmh / 3.6f;

    auto it = cfg_.highway_map.find(
        [&]() -> std::string {
            // Reverse-lookup class → highway string for default table
            for (const auto& [k, v] : cfg_.highway_map)
                if (v.klass == klass) return k;
            return "unclassified";
        }()
    );
    if (it != cfg_.highway_map.end())
        return it->second.default_speed_kmh / 3.6f;
    return 30.0f / 3.6f;
}

float OsmLoader::infer_capacity(int lanes, RoadClass klass) const {
    float cap_per_lane = 800.0f;
    for (const auto& [k, v] : cfg_.highway_map) {
        if (v.klass == klass) { cap_per_lane = v.capacity_per_lane_veh_h; break; }
    }
    return lanes * cap_per_lane;
}

bool OsmLoader::car_accessible(const std::string& access_tag,
                                  const std::string& highway_tag) const {
    if (!access_tag.empty()) {
        for (const auto& e : cfg_.car_exclude_tags)
            if (access_tag == e) return false;
        for (const auto& a : cfg_.car_access_tags)
            if (access_tag == a) return true;
        return false;
    }
    // No access tag: use highway class default
    auto it = cfg_.highway_map.find(highway_tag);
    if (it == cfg_.highway_map.end()) return false;
    RoadClass klass = it->second.klass;
    return klass <= RoadClass::Unclassified;  // exclude footway/cycleway/steps by default
}

// ── libosmium handlers ────────────────────────────────────────────────────────

// Pass 1 + 2 combined: libosmium's NodeLocationsForWays handler stores
// node coordinates so way handlers have access to them.
struct NomadWayHandler : osmium::handler::Handler {
    const OsmTagConfig& cfg;
    std::vector<RawEdge>& raw_edges;
    std::vector<std::string>& way_names;
    std::vector<OsmWayId>& way_ids_out;
    // Collect node coordinates here so build_graph can populate NodeData.
    // Key: OSM node ID → (lon, lat) as float.
    std::unordered_map<OsmNodeId, std::pair<float,float>>& coords;

    explicit NomadWayHandler(const OsmTagConfig& c,
                               std::vector<RawEdge>& re,
                               std::vector<std::string>& wn,
                               std::vector<OsmWayId>& wi,
                               std::unordered_map<OsmNodeId,
                                   std::pair<float,float>>& co)
        : cfg(c), raw_edges(re), way_names(wn), way_ids_out(wi), coords(co) {}

    void way(const osmium::Way& way) {
        const char* highway = way.tags()["highway"];
        if (!highway) return;

        auto it = cfg.highway_map.find(highway);
        if (it == cfg.highway_map.end()) return;

        const auto& defaults = it->second;

        // Access check
        const char* access = way.tags()["access"];
        const char* motor  = way.tags()["motor_vehicle"];
        const char* car    = way.tags()["motorcar"];
        std::string access_str = motor ? motor : (car ? car : (access ? access : ""));
        // Basic car accessible check
        bool accessible = true;
        for (const auto& ex : cfg.car_exclude_tags)
            if (access_str == ex) { accessible = false; break; }
        if (!accessible) return;

        // Speed
        const char* maxspeed = way.tags()["maxspeed"];
        float speed_ms = -1.0f;
        if (maxspeed) {
            float kmh = parse_maxspeed(maxspeed);
            if (kmh > 0) speed_ms = kmh / 3.6f;
        }
        if (speed_ms < 0) speed_ms = defaults.default_speed_kmh / 3.6f;

        // Lanes
        int lanes = defaults.default_lanes;
        const char* lanes_tag = way.tags()["lanes"];
        if (lanes_tag) {
            int l = 0;
            auto r = std::from_chars(lanes_tag, lanes_tag + strlen(lanes_tag), l);
            if (r.ec == std::errc() && l > 0) lanes = l;
        }

        // Capacity
        float capacity = lanes * defaults.capacity_per_lane_veh_h;

        // Oneway
        const char* oneway = way.tags()["oneway"];
        bool is_oneway = oneway && (std::string(oneway) == "yes" ||
                                     std::string(oneway) == "1"  ||
                                     std::string(oneway) == "true");
        bool reversed  = oneway && std::string(oneway) == "-1";

        // Flags
        uint8_t flags = 0;
        if (is_oneway || reversed) flags |= EdgeFlags::OneWay;
        const char* bridge = way.tags()["bridge"];
        if (bridge && std::string(bridge) == "yes") flags |= EdgeFlags::Bridge;
        const char* tunnel = way.tags()["tunnel"];
        if (tunnel && std::string(tunnel) == "yes") flags |= EdgeFlags::Tunnel;
        const char* junc = way.tags()["junction"];
        if (junc && std::string(junc) == "roundabout") flags |= EdgeFlags::Roundabout;

        // Name
        const char* name = way.tags()["name"];
        std::string name_str = name ? name : "";

        // Way metadata index
        uint16_t meta_idx = static_cast<uint16_t>(way_names.size() % 65535);
        way_names.push_back(name_str);
        way_ids_out.push_back(way.id());

        // Build edges from consecutive node pairs
        const auto& nodes = way.nodes();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto& n = nodes[i];
            // Store every referenced node's coordinates while we have them.
            if (n.location().valid()) {
                coords.emplace(n.ref(),
                    std::make_pair(static_cast<float>(n.location().lon()),
                                   static_cast<float>(n.location().lat())));
            }

            if (i + 1 >= nodes.size()) continue;
            const auto& nA = nodes[i];
            const auto& nB = nodes[i + 1];
            if (!nA.location().valid() || !nB.location().valid()) continue;

            // Haversine length
            constexpr float R = 6'371'000.0f;
            constexpr float deg = 3.14159265f / 180.0f;
            float lon1 = static_cast<float>(nA.location().lon());
            float lat1 = static_cast<float>(nA.location().lat());
            float lon2 = static_cast<float>(nB.location().lon());
            float lat2 = static_cast<float>(nB.location().lat());
            float dlat = (lat2 - lat1) * deg;
            float dlon = (lon2 - lon1) * deg;
            float a = std::sin(dlat/2)*std::sin(dlat/2) +
                       std::cos(lat1*deg)*std::cos(lat2*deg)*
                       std::sin(dlon/2)*std::sin(dlon/2);
            float length_m = 2.0f * R * std::asin(std::sqrt(std::max(0.0f, std::min(1.0f, a))));
            if (length_m < 0.1f) continue; // skip degenerate zero-length segments

            // Forward edge
            RawEdge fwd;
            fwd.way_id    = way.id();
            fwd.from_node = reversed ? nB.ref() : nA.ref();
            fwd.to_node   = reversed ? nA.ref() : nB.ref();
            fwd.length_m  = length_m;
            fwd.speed_ms  = speed_ms;
            fwd.capacity  = capacity;
            fwd.road_class= static_cast<uint8_t>(defaults.klass);
            fwd.flags     = flags;
            fwd.way_meta_idx = meta_idx;
            fwd.name      = name_str;
            raw_edges.push_back(fwd);

            // Reverse edge (if bidirectional)
            if (!is_oneway && !reversed) {
                RawEdge rev  = fwd;
                rev.from_node = nB.ref();
                rev.to_node   = nA.ref();
                rev.flags     |= EdgeFlags::Reversed;
                raw_edges.push_back(rev);
            }
        }
    }
};

// Pass 3: node metadata (signals, stop signs) and relation restrictions
struct NomadNodeHandler : osmium::handler::Handler {
    std::unordered_map<OsmNodeId, IntersectionMeta>& intersections;
    const SignalDefaults& signal_defaults;

    explicit NomadNodeHandler(std::unordered_map<OsmNodeId, IntersectionMeta>& im,
                               const SignalDefaults& sd)
        : intersections(im), signal_defaults(sd) {}

    void node(const osmium::Node& node) {
        const char* hw = node.tags()["highway"];
        if (!hw) return;
        std::string hw_str(hw);

        IntersectionType type = IntersectionType::Simple;
        if (hw_str == "traffic_signals")  type = IntersectionType::TrafficSignal;
        else if (hw_str == "stop")        type = IntersectionType::StopSign;
        else if (hw_str == "give_way")    type = IntersectionType::StopSign;
        else if (hw_str == "mini_roundabout") type = IntersectionType::MiniRoundabout;
        else return;

        IntersectionMeta meta;
        meta.osm_id = node.id();
        meta.type   = type;

        if (type == IntersectionType::TrafficSignal) {
            SignalPhase phase;
            phase.phase_id = 0;
            phase.cycle_s  = signal_defaults.cycle_s;
            phase.green_s  = signal_defaults.cycle_s * signal_defaults.green_frac;
            phase.movement_mask = 0xFFFF; // all movements
            meta.phases.push_back(phase);
        }

        intersections[node.id()] = std::move(meta);
    }
};

struct NomadRelationHandler : osmium::handler::Handler {
    std::vector<TurnRestriction>& restrictions;

    explicit NomadRelationHandler(std::vector<TurnRestriction>& r)
        : restrictions(r) {}

    void relation(const osmium::Relation& rel) {
        const char* type = rel.tags()["type"];
        if (!type || std::string(type) != "restriction") return;
        const char* restriction = rel.tags()["restriction"];
        if (!restriction) return;

        std::string r_str(restriction);
        TurnRestriction::Type rtype;
        if (r_str.substr(0, 3) == "no_")     rtype = TurnRestriction::Type::NoTurn;
        else if (r_str.substr(0, 5) == "only_") rtype = TurnRestriction::Type::OnlyTurn;
        else return;

        OsmWayId  from_way  = 0;
        OsmWayId  to_way    = 0;
        OsmNodeId via_node  = 0;

        for (const auto& member : rel.members()) {
            std::string role(member.role());
            if (role == "from" && member.type() == osmium::item_type::way)
                from_way = member.ref();
            else if (role == "to" && member.type() == osmium::item_type::way)
                to_way = member.ref();
            else if (role == "via" && member.type() == osmium::item_type::node)
                via_node = member.ref();
        }

        if (from_way && to_way && via_node) {
            restrictions.push_back({from_way, to_way, via_node, rtype});
        }
    }
};

// ── Graph builder ─────────────────────────────────────────────────────────────
OsmLoader::OsmLoader(OsmTagConfig cfg) : cfg_(std::move(cfg)) {}

Graph OsmLoader::build_graph(
    std::vector<RawEdge>& raw_edges,
    const std::unordered_map<OsmNodeId, std::pair<float,float>>& coords)
{
    // 1. Deduplicate OSM nodes → Graph NodeIds
    std::unordered_map<OsmNodeId, NodeId> node_map;
    node_map.reserve(raw_edges.size() * 2);

    for (const auto& e : raw_edges) {
        if (!node_map.count(e.from_node)) {
            NodeId id = static_cast<NodeId>(node_map.size());
            node_map[e.from_node] = id;
        }
        if (!node_map.count(e.to_node)) {
            NodeId id = static_cast<NodeId>(node_map.size());
            node_map[e.to_node] = id;
        }
    }

    uint32_t N = static_cast<uint32_t>(node_map.size());
    uint32_t E = static_cast<uint32_t>(raw_edges.size());

    // 2. Build NodeData array
    std::vector<NodeData> nodes(N);
    for (const auto& [osm_id, lid] : node_map) {
        auto it = coords.find(osm_id);
        if (it != coords.end()) {
            nodes[lid].lon = it->second.first;
            nodes[lid].lat = it->second.second;
        }
        nodes[lid].intersection_type = static_cast<uint8_t>(IntersectionType::Simple);
        nodes[lid].signal_phase_idx  = 0;
        nodes[lid].osm_node_idx      = lid; // placeholder
    }

    // Apply intersection metadata
    std::vector<SignalSchedule> signal_schedules;
    signal_schedules.push_back({0, 0, 0}); // index 0 = no signal
    for (auto& [osm_id, meta] : intersections_) {
        auto it = node_map.find(osm_id);
        if (it == node_map.end()) continue;
        NodeId lid = it->second;
        nodes[lid].intersection_type = static_cast<uint8_t>(meta.type);
        if (!meta.phases.empty()) {
            const auto& phase = meta.phases[0];
            uint8_t idx = static_cast<uint8_t>(
                std::min<std::size_t>(signal_schedules.size(), 254));
            signal_schedules.push_back({
                static_cast<uint16_t>(phase.cycle_s),
                static_cast<uint8_t>(phase.green_s / phase.cycle_s * 100),
                0
            });
            nodes[lid].signal_phase_idx = idx;
        }
    }

    // 3. Build CSR topology
    // Count out-degree per node
    std::vector<uint32_t> degree(N, 0);
    for (const auto& e : raw_edges)
        ++degree[node_map.at(e.from_node)];

    // Prefix sum → row_ptr
    std::vector<uint32_t> row_ptr(N + 1, 0);
    for (uint32_t i = 0; i < N; ++i) row_ptr[i + 1] = row_ptr[i] + degree[i];

    // Fill col_idx and edges
    std::vector<EdgeId>   col_idx(E);
    std::vector<EdgeData> edges(E);
    std::vector<uint32_t> fill_pos = row_ptr; // current fill pointer per row

    // Geometry sidecar
    std::vector<uint32_t>                geom_ptr(E + 1, 0);
    std::vector<std::pair<float, float>> geom_coords;

    // Way metadata
    std::vector<OsmWayId>    way_ids_table;
    std::vector<std::string> way_names_table;

    for (EdgeId eid = 0; eid < E; ++eid) {
        const RawEdge& re = raw_edges[eid];
        NodeId src = node_map.at(re.from_node);
        uint32_t pos = fill_pos[src]++;

        col_idx[pos] = eid;  // CSR pos → EdgeId

        // Store edge data at EdgeId (eid), not at CSR position (pos).
        // edges[eid] is what all callers use via out_edges(u) → edges[eid].
        auto& ed = edges[eid];
        ed.target          = node_map.at(re.to_node);
        ed.length_m        = re.length_m;
        ed.free_flow_speed = re.speed_ms;
        ed.road_class      = re.road_class;
        ed.flags           = re.flags;
        ed.way_meta_idx    = re.way_meta_idx;

        // Intersection capacity derating (HCM/TCQSM, see intersection.hpp) --
        // applied per effective_capacity()'s own doc comment ("at the edge's
        // source node"): a signal/stop/roundabout at the node this edge
        // LEAVES FROM constrains how much of re.capacity's mid-block
        // throughput actually gets discharged onto it. Nodes with no
        // intersection metadata (the common case -- only traffic_signals/
        // stop/give_way/mini_roundabout tagged nodes are recorded) fall
        // through to effective_capacity()'s IntersectionType::Simple default,
        // which returns base_capacity unchanged.
        float capacity = re.capacity;
        if (auto it = intersections_.find(re.from_node); it != intersections_.end()) {
            const SignalPhase* phase =
                it->second.phases.empty() ? nullptr : &it->second.phases[0];
            capacity = effective_capacity(capacity, it->second.type, phase);
        }
        ed.capacity = capacity;

        // Geometry: no intermediate points at segment level (polyline points
        // are represented by consecutive edges), so geom_ptr[eid] = eid offset.
        geom_ptr[pos + 1] = geom_ptr[pos]; // no intermediate shape points
    }

    Graph g;
    g.row_ptr  = std::move(row_ptr);
    g.col_idx  = std::move(col_idx);
    g.edges    = std::move(edges);
    g.nodes    = std::move(nodes);
    g.geom_ptr = std::move(geom_ptr);
    g.signal_schedules = std::move(signal_schedules);

    spdlog::info("Graph built: {} nodes, {} edges", g.num_nodes(), g.num_edges());
    return g;
}

Graph OsmLoader::load(const std::filesystem::path& osm_pbf) {
    // SparseMemArray: stores only the node IDs actually referenced by ways.
    // This is safe for any city size. FlexMem can pre-allocate a dense array
    // sized to the max node ID (billions for Italy) → segfault / OOM.
    using Index = osmium::index::map::SparseMemArray<
        osmium::unsigned_object_id_type, osmium::Location>;
    using LocationHandler = osmium::handler::NodeLocationsForWays<Index>;

    spdlog::info("Loading OSM file: {}", osm_pbf.string());

    osmium::io::File input_file{osm_pbf.string()};

    std::vector<RawEdge>     raw_edges;
    std::vector<std::string> way_names_out;
    std::vector<OsmWayId>    way_ids_out;
    // Populated by NomadWayHandler::way() while node locations are available.
    std::unordered_map<OsmNodeId, std::pair<float,float>> coord_map;

    // Pass 1+2: node locations + way geometry (single reader pass)
    {
        Index index;
        LocationHandler location_handler{index};
        NomadWayHandler  way_handler{cfg_, raw_edges, way_names_out,
                                      way_ids_out, coord_map};
        NomadNodeHandler node_handler{intersections_, cfg_.signal_defaults};

        osmium::io::Reader reader{input_file,
            osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
        osmium::apply(reader, location_handler, way_handler, node_handler);
        reader.close();
    }

    spdlog::info("Parsed {} raw edges, {} node coords, {} intersections",
                  raw_edges.size(), coord_map.size(), intersections_.size());

    // Pass 2: restriction relations
    {
        NomadRelationHandler rel_handler{turn_restrictions_};
        osmium::io::Reader reader{input_file, osmium::osm_entity_bits::relation};
        osmium::apply(reader, rel_handler);
        reader.close();
    }

    spdlog::info("Parsed {} turn restrictions", turn_restrictions_.size());

    return build_graph(raw_edges, coord_map);
}

Graph OsmLoader::load_and_clean(const std::filesystem::path& osm_pbf,
                                   bool simplify_topology) {
    Graph g = load(osm_pbf);
    NetworkCleaner cleaner({5, simplify_topology, true, true, 20.0f});
    return cleaner.clean(std::move(g));
}

} // namespace nomad
