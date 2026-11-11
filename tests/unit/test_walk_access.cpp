// Walk access: OSM foot=no / foot=use_sidepath, and walk access of edges
// merged by NetworkCleaner. People on foot must never be routed onto
// motorways/trunk roads or onto ways where walking is forbidden, while the
// car graph must stay exactly as before.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <nomad/core/graph.hpp>
#include <nomad/network/network_cleaner.hpp>
#include <nomad/network/osm_loader.hpp>
#include <nomad/routing/astar_router.hpp>

#include <osmium/builder/attr.hpp>
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>

#include <filesystem>
#include <string>
#include <vector>

using namespace nomad;

// ── edge_accessible ──────────────────────────────────────────────────────────
TEST_CASE("edge_accessible: NoFoot blocks walk only", "[walk]") {
    const auto P = static_cast<uint8_t>(RoadClass::Primary);
    const auto C = static_cast<uint8_t>(RoadClass::Cycleway);
    REQUIRE_FALSE(edge_accessible(P, EdgeFlags::NoFoot, AgentMode::Walk));
    REQUIRE(edge_accessible(P, EdgeFlags::NoFoot, AgentMode::Car));
    REQUIRE(edge_accessible(C, EdgeFlags::NoFoot, AgentMode::Bike));
    REQUIRE(edge_accessible(P, 0, AgentMode::Walk));
    // class rule still applies
    REQUIRE_FALSE(edge_accessible(static_cast<uint8_t>(RoadClass::Motorway), 0, AgentMode::Walk));
}

// ── A* respects NoFoot ───────────────────────────────────────────────────────
// 0 → 2 direct (short, NoFoot) and 0 → 1 → 2 detour (long, walkable).
static Graph make_triangle(uint8_t direct_flags) {
    Graph g;
    g.nodes.resize(3);
    g.nodes[0] = {0.000f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {0.005f, 0.005f, 0, 0, 0, 1};
    g.nodes[2] = {0.010f, 0.0f, 0, 0, 0, 2};
    auto e = [](NodeId t, float len, uint8_t fl) {
        EdgeData d{};
        d.target = t; d.length_m = len; d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(RoadClass::Primary); d.flags = fl;
        return d;
    };
    g.row_ptr = {0, 2, 3, 3};
    g.edges   = {e(2, 1000.0f, direct_flags), e(1, 800.0f, 0), e(2, 800.0f, 0)};
    g.col_idx = {0, 1, 2};
    g.geom_ptr.assign(4, 0);
    return g;
}

TEST_CASE("AStarRouter: walk avoids NoFoot edge, car does not", "[walk][routing]") {
    Graph g = make_triangle(EdgeFlags::NoFoot);
    AStarRouter router(g);
    Route rc = router.route(RoutingRequest{0, 2, 0.0, AgentMode::Car});
    Route rw = router.route(RoutingRequest{0, 2, 0.0, AgentMode::Walk});
    REQUIRE(rc.is_valid);
    REQUIRE(rw.is_valid);
    REQUIRE(rc.edges == std::vector<EdgeId>{0});
    REQUIRE(rw.edges == std::vector<EdgeId>{1, 2});
}

// ── NetworkCleaner merges keep walk access conservative ──────────────────────
// One-way chain 0 → 1 → 2: primary then motorway_link. After contraction the
// merged edge keeps the first segment's class (car graph unchanged) but must
// not be walkable, since crossing it means walking on the motorway link.
static Graph make_chain2(RoadClass c1, uint8_t f1, RoadClass c2, uint8_t f2) {
    Graph g;
    g.nodes.resize(3);
    g.nodes[0] = {0.0f, 0.0f, 0, 0, 0, 0};
    g.nodes[1] = {1.0f, 0.0f, 0, 0, 0, 1};
    g.nodes[2] = {2.0f, 0.0f, 0, 0, 0, 2};
    g.row_ptr = {0, 1, 2, 2};
    g.col_idx = {0, 1};
    g.edges.resize(2);
    for (int i = 0; i < 2; ++i) {
        EdgeData d{};
        d.target = static_cast<NodeId>(i + 1); d.length_m = 100.0f;
        d.free_flow_speed = 10.0f; d.capacity = 1600.0f;
        d.road_class = static_cast<uint8_t>(i == 0 ? c1 : c2);
        d.flags = i == 0 ? f1 : f2;
        g.edges[i] = d;
    }
    g.geom_ptr.assign(3, 0);
    return g;
}

static NetworkCleaner::Config permissive_cfg() {
    NetworkCleaner::Config cfg;
    cfg.min_component_size = 1;
    return cfg;
}

TEST_CASE("NetworkCleaner: merging onto a non-walkable class sets NoFoot", "[walk][network]") {
    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(make_chain2(RoadClass::Primary, 0, RoadClass::MotorwayLink, 0));
    REQUIRE(out.num_edges() == 1);
    const auto& m = out.edges[0];
    REQUIRE(m.road_class == static_cast<uint8_t>(RoadClass::Primary));  // car graph unchanged
    REQUIRE_THAT(m.length_m, Catch::Matchers::WithinAbs(200.0f, 0.01f));
    REQUIRE_FALSE(edge_accessible(m.road_class, m.flags, AgentMode::Walk));
    REQUIRE(edge_accessible(m.road_class, m.flags, AgentMode::Car));
}

TEST_CASE("NetworkCleaner: NoFoot on either merged segment propagates", "[walk][network]") {
    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(make_chain2(RoadClass::Primary, 0, RoadClass::Primary, EdgeFlags::NoFoot));
    REQUIRE(out.num_edges() == 1);
    REQUIRE((out.edges[0].flags & EdgeFlags::NoFoot) != 0);
}

TEST_CASE("NetworkCleaner: walkable chain stays walkable", "[walk][network]") {
    NetworkCleaner cleaner(permissive_cfg());
    Graph out = cleaner.clean(make_chain2(RoadClass::Primary, 0, RoadClass::Residential, 0));
    REQUIRE(out.num_edges() == 1);
    REQUIRE(edge_accessible(out.edges[0].road_class, out.edges[0].flags, AgentMode::Walk));
}

// ── OsmLoader reads foot=no / foot=use_sidepath ──────────────────────────────
// Tiny PBF written with libosmium: four separate 2-node ways, each with its
// own nodes, so edges can be told apart by their source node.
namespace {
using namespace osmium::builder::attr;

std::filesystem::path write_pbf(const std::string& name) {
    const osmium::Location loc0{2.60, 39.50};
    osmium::memory::Buffer buf{1024 * 64, osmium::memory::Buffer::auto_grow::yes};
    // way k uses nodes 10k+1, 10k+2
    struct W { int id; std::vector<std::pair<const char*, const char*>> tags; };
    std::vector<W> ways = {
        {1, {{"highway", "primary"}, {"oneway", "yes"}, {"foot", "no"}}},
        {2, {{"highway", "primary"}, {"oneway", "yes"}}},
        {3, {{"highway", "cycleway"}, {"foot", "no"}}},
        {4, {{"highway", "secondary"}, {"foot", "use_sidepath"}}},
    };
    for (const auto& w : ways) {
        for (int j = 1; j <= 2; ++j) {
            osmium::builder::add_node(buf, _id(10 * w.id + j), _version(1),
                _location(osmium::Location{2.60 + 0.01 * w.id + 0.002 * j, 39.50}));
        }
    }
    for (const auto& w : ways) {
        osmium::builder::add_way(buf, _id(w.id), _version(1),
            _nodes({10 * w.id + 1, 10 * w.id + 2}), _tags(w.tags));
    }
    (void)loc0;
    auto path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(path);
    osmium::io::Writer writer{path.string()};
    writer(std::move(buf));
    writer.close();
    return path;
}
} // namespace

TEST_CASE("OsmLoader: foot=no and foot=use_sidepath make edges non-walkable", "[walk][osm]") {
    auto path = write_pbf("nomad_test_walk_access.osm.pbf");
    OsmLoader loader;
    Graph g = loader.load(path);

    // Classify edges by the x-coordinate of their source node (way k sits
    // around lon 2.60 + 0.01 k).
    struct Count { int car = 0, walk = 0; };
    Count c[5];
    for (uint32_t u = 0; u < g.num_nodes(); ++u) {
        int k = static_cast<int>(std::lround((g.nodes[u].lon - 2.60) / 0.01 - 0.3));
        for (EdgeId e : g.out_edges(u)) {
            const auto& ed = g.edges[e];
            if (edge_accessible(ed.road_class, ed.flags, AgentMode::Car))  ++c[k].car;
            if (edge_accessible(ed.road_class, ed.flags, AgentMode::Walk)) ++c[k].walk;
        }
    }
    // primary oneway foot=no: one car edge, no walk edge (no contraflow footway)
    REQUIRE(c[1].car == 1);
    REQUIRE(c[1].walk == 0);
    // primary oneway: car one way, walk both ways (contraflow footway)
    REQUIRE(c[2].car == 1);
    REQUIRE(c[2].walk == 2);
    // cycleway foot=no: not walkable
    REQUIRE(c[3].walk == 0);
    // secondary foot=use_sidepath: cars both ways, nobody walks on it
    REQUIRE(c[4].car == 2);
    REQUIRE(c[4].walk == 0);
    std::filesystem::remove(path);
}
