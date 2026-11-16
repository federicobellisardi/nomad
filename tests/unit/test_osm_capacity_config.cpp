// Tag-config overrides for capacity: per-lane capacity by highway class
// (capacity_per_lane_veh_h) and the green share of mid-block signals
// (midblock_signal_green_frac). A mid-block signal is one tagged as a
// pedestrian crossing, or on a node with at most two car neighbours.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/network/intersection.hpp>
#include <nomad/network/osm_loader.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace nomad;
using Catch::Matchers::WithinAbs;

namespace {
using namespace osmium::builder::attr;
namespace fs = std::filesystem;

// node id -> (lon, lat); node ids are unique per position
const std::map<int, std::pair<double, double>> kNodes = {
    // A: west-east primary through 11-12-13, signal at 12, nothing crosses (mid-block)
    {11, {2.600, 39.50}}, {12, {2.602, 39.50}}, {13, {2.604, 39.50}},
    // B: west-east 21-22-23 crossed by north-south 31-22-33, signal at 22 (intersection)
    {21, {2.610, 39.50}}, {22, {2.612, 39.50}}, {23, {2.614, 39.50}},
    {31, {2.612, 39.498}}, {33, {2.612, 39.502}},
    // D: same as B but the signal is tagged as a pedestrian crossing
    {41, {2.620, 39.50}}, {42, {2.622, 39.50}}, {43, {2.624, 39.50}},
    {51, {2.622, 39.498}}, {53, {2.622, 39.502}},
};

fs::path write_pbf() {
    osmium::memory::Buffer buf{1024 * 64, osmium::memory::Buffer::auto_grow::yes};
    for (const auto& [id, ll] : kNodes) {
        const osmium::Location loc{ll.first, ll.second};
        if (id == 12 || id == 22)
            osmium::builder::add_node(buf, _id(id), _version(1), _location(loc),
                                      _tags({{"highway", "traffic_signals"}}));
        else if (id == 42)
            osmium::builder::add_node(buf, _id(id), _version(1), _location(loc),
                                      _tags({{"highway", "traffic_signals"}, {"crossing", "traffic_signals"}}));
        else
            osmium::builder::add_node(buf, _id(id), _version(1), _location(loc));
    }
    auto way = [&](int id, std::initializer_list<int> nodes) {
        std::vector<osmium::object_id_type> v(nodes.begin(), nodes.end());
        osmium::builder::add_way(buf, _id(id), _version(1), _nodes(v), _tags({{"highway", "primary"}}));
    };
    way(1, {11, 12, 13}); way(2, {21, 22, 23}); way(3, {31, 22, 33});
    way(4, {41, 42, 43}); way(5, {51, 42, 53});
    auto path = fs::temp_directory_path() / "nomad_test_osm_capacity_config.osm.pbf";
    fs::remove(path);
    osmium::io::Writer writer{path.string()};
    writer(std::move(buf));
    writer.close();
    return path;
}

int node_at(const Graph& g, NodeId u) {
    for (const auto& [id, ll] : kNodes)
        if (std::abs(g.nodes[u].lon - ll.first) < 1e-5 && std::abs(g.nodes[u].lat - ll.second) < 1e-5) return id;
    return -1;
}

// capacity of the car edge between two OSM node ids
std::map<std::pair<int, int>, float> caps(const Graph& g) {
    std::map<std::pair<int, int>, float> out;
    for (uint32_t u = 0; u < g.num_nodes(); ++u)
        for (EdgeId e : g.out_edges(u)) {
            const auto& ed = g.edges[e];
            if (!edge_accessible(ed.road_class, ed.flags, AgentMode::Car)) continue;
            out[{node_at(g, u), node_at(g, ed.target)}] = ed.capacity;
        }
    return out;
}

fs::path write_yaml(const std::string& body) {
    auto p = fs::temp_directory_path() / "nomad_test_osm_capacity_config.yaml";
    std::ofstream(p) << body;
    return p;
}
} // namespace

TEST_CASE("OsmTagConfig: per-lane capacity overrides and mid-block signal green", "[osm][intersection][capacity]") {
    auto pbf = write_pbf();
    const float base = 2 * 1800.0f;   // untagged two-way primary: 2 lanes per direction x 1800
    const float sig  = base * 0.45f * CapacityDerate::kSignalSaturation;
    const float mid  = base * 0.75f * CapacityDerate::kSignalSaturation;

    SECTION("capacity override only: every signal keeps the intersection green") {
        auto y = write_yaml("capacity_per_lane_veh_h:\n  primary: 1800\n");
        Graph g = OsmLoader(OsmTagConfig::load_yaml(y)).load(pbf);
        auto c = caps(g);
        REQUIRE_THAT(c.at({11, 12}), WithinAbs(sig, 1e-2f));
        REQUIRE_THAT(c.at({21, 22}), WithinAbs(sig, 1e-2f));
        REQUIRE_THAT(c.at({12, 13}), WithinAbs(base, 1e-2f));   // departure not derated
        fs::remove(y);
    }
    SECTION("mid-block green: pass-through and crossing-tagged signals, not intersections") {
        auto y = write_yaml("capacity_per_lane_veh_h:\n  primary: 1800\nmidblock_signal_green_frac: 0.75\n");
        Graph g = OsmLoader(OsmTagConfig::load_yaml(y)).load(pbf);
        auto c = caps(g);
        REQUIRE_THAT(c.at({11, 12}), WithinAbs(mid, 1e-2f));   // A: pass-through
        REQUIRE_THAT(c.at({13, 12}), WithinAbs(mid, 1e-2f));
        REQUIRE_THAT(c.at({21, 22}), WithinAbs(sig, 1e-2f));   // B: real intersection
        REQUIRE_THAT(c.at({31, 22}), WithinAbs(sig, 1e-2f));
        REQUIRE_THAT(c.at({41, 42}), WithinAbs(mid, 1e-2f));   // D: tagged crossing
        REQUIRE_THAT(c.at({51, 42}), WithinAbs(mid, 1e-2f));
        fs::remove(y);
    }
    SECTION("defaults unchanged without the YAML keys") {
        Graph g = OsmLoader().load(pbf);
        auto c = caps(g);
        REQUIRE_THAT(c.at({11, 12}), WithinAbs(2 * 1600.0f * 0.45f * CapacityDerate::kSignalSaturation, 1e-2f));
    }
    fs::remove(pbf);
}
