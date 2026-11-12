// Implied one-way in OSM: junction=roundabout, junction=circular and
// highway=motorway are one-way unless tagged oneway=no. Without this every
// roundabout became a two-way ring (cars could circulate against the flow).
#include <catch2/catch_test_macros.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/network/osm_loader.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace nomad;

namespace {
using namespace osmium::builder::attr;

struct W { int id; std::vector<std::pair<const char*, const char*>> tags; };

const std::vector<W> kWays = {
    {1, {{"highway", "residential"}, {"junction", "roundabout"}}},
    {2, {{"highway", "residential"}, {"junction", "circular"}}},
    {3, {{"highway", "residential"}, {"junction", "roundabout"}, {"oneway", "no"}}},
    {4, {{"highway", "motorway"}}},
    {5, {{"highway", "residential"}}},
    {6, {{"highway", "primary"}, {"junction", "roundabout"}}},
    {7, {{"highway", "residential"}, {"junction", "roundabout"}, {"oneway", "-1"}}},
};

std::filesystem::path write_pbf(const std::string& name) {
    osmium::memory::Buffer buf{1024 * 64, osmium::memory::Buffer::auto_grow::yes};
    for (const auto& w : kWays)
        for (int j = 1; j <= 2; ++j)
            osmium::builder::add_node(buf, _id(10 * w.id + j), _version(1),
                _location(osmium::Location{2.60 + 0.01 * w.id + 0.002 * j, 39.50}));
    for (const auto& w : kWays)
        osmium::builder::add_way(buf, _id(w.id), _version(1),
            _nodes({10 * w.id + 1, 10 * w.id + 2}), _tags(w.tags));
    auto path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(path);
    osmium::io::Writer writer{path.string()};
    writer(std::move(buf));
    writer.close();
    return path;
}
} // namespace

TEST_CASE("OsmLoader: roundabouts and motorways are implied one-way", "[osm][oneway]") {
    auto path = write_pbf("nomad_test_implied_oneway.osm.pbf");
    OsmLoader loader;
    Graph g = loader.load(path);

    struct Count { int car = 0, walk = 0; bool forward = false; };
    Count c[8];
    for (uint32_t u = 0; u < g.num_nodes(); ++u) {
        int k = static_cast<int>(std::lround((g.nodes[u].lon - 2.60) / 0.01 - 0.3));
        for (EdgeId e : g.out_edges(u)) {
            const auto& ed = g.edges[e];
            if (edge_accessible(ed.road_class, ed.flags, AgentMode::Car)) {
                ++c[k].car;
                // forward = from the way's first node (smaller lon) to its second
                if (g.nodes[ed.target].lon > g.nodes[u].lon) c[k].forward = true;
            }
            if (edge_accessible(ed.road_class, ed.flags, AgentMode::Walk)) ++c[k].walk;
        }
    }
    REQUIRE(c[1].car == 1);            // roundabout
    REQUIRE(c[1].forward);
    REQUIRE(c[2].car == 1);            // circular
    REQUIRE(c[3].car == 2);            // explicit oneway=no wins
    REQUIRE(c[4].car == 1);            // motorway
    REQUIRE(c[5].car == 2);            // control: plain two-way street
    REQUIRE(c[6].car == 1);            // primary roundabout: one-way for cars,
    REQUIRE(c[6].walk == 2);           // walkable both ways (contraflow footway)
    REQUIRE(c[7].car == 1);            // oneway=-1 still reverses
    REQUIRE_FALSE(c[7].forward);
    std::filesystem::remove(path);
}
