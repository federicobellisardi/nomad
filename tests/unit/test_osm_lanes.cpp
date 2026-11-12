// Per-direction lane count from OSM, stored in Graph::edge_lanes (graph v2).
//
// OSM `lanes` is the total over both directions of a two-way way. Before this
// change every directional edge got `lanes * capacity_per_lane`, doubling the
// capacity of tagged two-way roads, and the LTM inferred storage lanes as
// capacity / 1600 (biased by the per-class capacity and by signal derating).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/network/network_cleaner.hpp>
#include <nomad/network/osm_loader.hpp>
#include <nomad/traffic/ltm_model.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace nomad;
using Catch::Matchers::WithinAbs;

namespace {
using namespace osmium::builder::attr;

struct W { int id; std::vector<std::pair<const char*, const char*>> tags; };

const std::vector<W> kWays = {
    {1, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}}},
    {2, {{"highway", "primary"}, {"lanes", "4"}}},
    {3, {{"highway", "tertiary"}, {"lanes", "3"}}},
    {4, {{"highway", "primary"}, {"lanes", "3"}, {"lanes:forward", "2"}, {"lanes:backward", "1"}}},
    {5, {{"highway", "primary"}, {"lanes", "3"}, {"lanes:forward", "2"}}},
    {6, {{"highway", "primary"}}},
    {7, {{"highway", "primary"}, {"lanes", "3"}, {"lanes:both_ways", "1"}}},
    {8, {{"highway", "motorway"}, {"lanes", "3"}}},
    {9, {{"highway", "primary"}, {"oneway", "-1"}, {"lanes", "2"}}},
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

struct Dir { float lanes = -1.0f, cap = -1.0f; };
struct WayLanes { Dir fwd, bwd; };
} // namespace

TEST_CASE("OsmLoader: per-direction lanes and capacity", "[osm][lanes]") {
    auto path = write_pbf("nomad_test_osm_lanes.osm.pbf");
    OsmLoader loader;
    Graph g = loader.load(path);
    REQUIRE(g.edge_lanes.size() == g.num_edges());

    WayLanes w[10];
    for (uint32_t u = 0; u < g.num_nodes(); ++u) {
        int k = static_cast<int>(std::lround((g.nodes[u].lon - 2.60) / 0.01 - 0.3));
        for (EdgeId e : g.out_edges(u)) {
            const auto& ed = g.edges[e];
            if (!edge_accessible(ed.road_class, ed.flags, AgentMode::Car)) continue;
            Dir& d = (g.nodes[ed.target].lon > g.nodes[u].lon) ? w[k].fwd : w[k].bwd;
            d.lanes = g.edge_lanes[e];
            d.cap   = ed.capacity;
        }
    }
    // one-way: the whole tag is one direction
    REQUIRE_THAT(w[1].fwd.lanes, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(w[1].fwd.cap,   WithinAbs(3 * 1600.0f, 1e-3f));
    REQUIRE(w[1].bwd.lanes < 0.0f);
    // two-way lanes=4: 2 per direction (was 4 each)
    REQUIRE_THAT(w[2].fwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(w[2].bwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(w[2].fwd.cap,   WithinAbs(2 * 1600.0f, 1e-3f));
    // odd total, no direction tags: split evenly
    REQUIRE_THAT(w[3].fwd.lanes, WithinAbs(1.5f, 1e-6f));
    REQUIRE_THAT(w[3].bwd.lanes, WithinAbs(1.5f, 1e-6f));
    REQUIRE_THAT(w[3].fwd.cap,   WithinAbs(1.5f * 800.0f, 1e-3f));
    // explicit lanes:forward / lanes:backward
    REQUIRE_THAT(w[4].fwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(w[4].bwd.lanes, WithinAbs(1.0f, 1e-6f));
    // only lanes:forward: backward = total - forward
    REQUIRE_THAT(w[5].fwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(w[5].bwd.lanes, WithinAbs(1.0f, 1e-6f));
    // untagged: class default per direction (unchanged behaviour)
    REQUIRE_THAT(w[6].fwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE_THAT(w[6].bwd.lanes, WithinAbs(2.0f, 1e-6f));
    // centre turn lane excluded from the split
    REQUIRE_THAT(w[7].fwd.lanes, WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(w[7].bwd.lanes, WithinAbs(1.0f, 1e-6f));
    // motorway (implied one-way)
    REQUIRE_THAT(w[8].fwd.lanes, WithinAbs(3.0f, 1e-6f));
    REQUIRE_THAT(w[8].fwd.cap,   WithinAbs(3 * 2000.0f, 1e-3f));
    // oneway=-1: the single (reversed) direction carries all lanes
    REQUIRE_THAT(w[9].bwd.lanes, WithinAbs(2.0f, 1e-6f));
    REQUIRE(w[9].fwd.lanes < 0.0f);
    std::filesystem::remove(path);
}

namespace {
EdgeData mk(NodeId t, float len, float cap) {
    EdgeData d{};
    d.target = t; d.length_m = len; d.free_flow_speed = 20.0f; d.capacity = cap;
    d.road_class = static_cast<uint8_t>(RoadClass::Motorway);
    return d;
}
Graph chain(float l0, float l1, bool with_lanes, float len0 = 100.0f) {
    Graph g;
    g.nodes.resize(3);
    for (int i = 0; i < 3; ++i) g.nodes[i] = {static_cast<float>(i) * 0.01f, 0.0f, 0, 0, 0, static_cast<uint32_t>(i)};
    g.row_ptr = {0, 1, 2, 2};
    g.col_idx = {0, 1};
    g.edges = {mk(1, len0, l0 * 2000.0f), mk(2, 300.0f, l1 * 2000.0f)};
    if (with_lanes) g.edge_lanes = {l0, l1};
    g.geom_ptr.assign(3, 0);
    return g;
}
} // namespace

TEST_CASE("Graph IO: edge_lanes round-trips (v2)", "[graph][lanes]") {
    Graph g = chain(3.0f, 2.0f, true);
    auto p = std::filesystem::temp_directory_path() / "nomad_test_lanes_v2.bin";
    g.save(p);
    Graph h = Graph::load(p);
    REQUIRE(h.edge_lanes == g.edge_lanes);
    std::filesystem::remove(p);
}

TEST_CASE("Graph IO: a v1 graph still loads, with no lane data", "[graph][lanes]") {
    // Minimal v1 file: magic, version 1, N=0, E=0, nine empty vectors and an
    // empty string table (row_ptr, col_idx, edges, nodes, geom_ptr,
    // geom_coords, osm_node_ids, way_ids, way_names, signal_schedules).
    auto p = std::filesystem::temp_directory_path() / "nomad_test_lanes_v1.bin";
    {
        std::ofstream out(p, std::ios::binary);
        out.write("NOMG", 4);
        const uint32_t ver = 1, zero = 0;
        out.write(reinterpret_cast<const char*>(&ver), 4);
        for (int i = 0; i < 2 + 10; ++i) out.write(reinterpret_cast<const char*>(&zero), 4);
    }
    Graph h = Graph::load(p);
    REQUIRE(h.num_edges() == 0);
    REQUIRE(h.edge_lanes.empty());
    std::filesystem::remove(p);
}

TEST_CASE("NetworkCleaner: merged edge keeps length-weighted lanes", "[network][lanes]") {
    NetworkCleaner::Config cfg; cfg.min_component_size = 1;
    NetworkCleaner cleaner(cfg);
    Graph out = cleaner.clean(chain(3.0f, 2.0f, true));
    REQUIRE(out.num_edges() == 1);
    REQUIRE(out.edge_lanes.size() == 1);
    // (100*3 + 300*2) / 400 = 2.25: total storage (length x lanes) is conserved
    REQUIRE_THAT(out.edge_lanes[0], WithinAbs(2.25f, 1e-5f));
    REQUIRE_THAT(out.edges[0].capacity, WithinAbs(2 * 2000.0f, 1e-3f)); // bottleneck
}

TEST_CASE("LTM: storage uses graph lanes when present, capacity/1600 otherwise", "[traffic][lanes]") {
    // e0: 101 m, 3 lanes, motorway capacity 6000 veh/h (length chosen so the
    // storage is not an integer and float rounding cannot shift the count).
    Graph with = chain(3.0f, 2.0f, true, 101.0f);
    Graph without = chain(3.0f, 2.0f, false, 101.0f);
    LtmTrafficModel::Config c;
    LtmTrafficModel m1(with, c), m0(without, c);
    // jam density 1/7.5 veh/m: 101*3/7.5 = 40.4 -> 41 admitted;
    // legacy 101*(6000/1600)/7.5 = 50.5 -> 51 admitted
    int n1 = 0; while (m1.has_capacity(0)) m1.on_enter(0, n1++, 0.0);
    int n0 = 0; while (m0.has_capacity(0)) m0.on_enter(0, n0++, 0.0);
    REQUIRE(n1 == 41);
    REQUIRE(n0 == 51);
}
