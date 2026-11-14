// Intersection capacity derating (signals, stop/give_way, mini-roundabouts).
//
// The traffic models enforce an edge's capacity where vehicles LEAVE it, i.e.
// at its target node (LTM discharge cap at on_exit). A control node therefore
// limits the edges that ARRIVE at it: the approaches to a signal, the minor
// approach at a give_way. Derating the edges that leave the node (the original
// behaviour) put the constraint one node downstream, on every departure.
// OSM direction tags (traffic_signals:direction, direction = forward|backward,
// relative to the way's node order) restrict the derate to one approach.
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
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace nomad;
using Catch::Matchers::WithinAbs;

namespace {
using namespace osmium::builder::attr;
using Tags = std::vector<std::pair<const char*, const char*>>;

// Each way k is a two-way primary (1600 veh/h per lane, 2 lanes per direction)
// through three nodes 10k+1 -> 10k+2 -> 10k+3, west to east; the middle node
// carries the control tags.
const std::vector<std::pair<int, Tags>> kWays = {
    {1, {{"highway", "traffic_signals"}}},
    {2, {{"highway", "traffic_signals"}, {"traffic_signals:direction", "forward"}}},
    {3, {{"highway", "give_way"}, {"direction", "backward"}}},
    {4, {{"highway", "stop"}}},
};

std::filesystem::path write_pbf(const std::string& name) {
    osmium::memory::Buffer buf{1024 * 64, osmium::memory::Buffer::auto_grow::yes};
    for (const auto& [k, tags] : kWays)
        for (int j = 1; j <= 3; ++j) {
            const osmium::Location loc{2.60 + 0.01 * k + 0.002 * j, 39.50};
            if (j == 2)
                osmium::builder::add_node(buf, _id(10 * k + j), _version(1), _location(loc), _tags(tags));
            else
                osmium::builder::add_node(buf, _id(10 * k + j), _version(1), _location(loc));
        }
    for (const auto& [k, tags] : kWays)
        osmium::builder::add_way(buf, _id(k), _version(1),
            _nodes({10 * k + 1, 10 * k + 2, 10 * k + 3}), _tags({{"highway", "primary"}}));
    auto path = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove(path);
    osmium::io::Writer writer{path.string()};
    writer(std::move(buf));
    writer.close();
    return path;
}

// capacity of the car edge from node position i to node position j of way k
// (positions 1..3 west to east), keyed by (k, i, j)
std::map<std::tuple<int, int, int>, float> edge_caps(const Graph& g) {
    std::map<std::tuple<int, int, int>, float> out;
    auto pos = [&](NodeId u, int& k, int& j) {
        const double x = (g.nodes[u].lon - 2.60) / 0.002;   // = 5k + j
        const int v = static_cast<int>(std::lround(x));
        k = v / 5; j = v % 5;
    };
    for (uint32_t u = 0; u < g.num_nodes(); ++u)
        for (EdgeId e : g.out_edges(u)) {
            const auto& ed = g.edges[e];
            if (!edge_accessible(ed.road_class, ed.flags, AgentMode::Car)) continue;
            int k, i, k2, j;
            pos(u, k, i); pos(ed.target, k2, j);
            out[{k, i, j}] = ed.capacity;
        }
    return out;
}
} // namespace

TEST_CASE("OsmLoader: control-node derate applies to the edges arriving at the node",
          "[osm][intersection]") {
    auto path = write_pbf("nomad_test_osm_derating.osm.pbf");
    OsmLoader loader;
    Graph g = loader.load(path);
    auto cap = edge_caps(g);
    const float base = 2 * 1600.0f;
    const float sig  = base * 0.45f * CapacityDerate::kSignalSaturation;
    const float stop = base * CapacityDerate::kStopSignDerate;

    // way 1, signal, no direction: both approaches derated, departures not
    REQUIRE_THAT(cap.at({1, 1, 2}), WithinAbs(sig, 1e-2f));
    REQUIRE_THAT(cap.at({1, 3, 2}), WithinAbs(sig, 1e-2f));
    REQUIRE_THAT(cap.at({1, 2, 3}), WithinAbs(base, 1e-2f));
    REQUIRE_THAT(cap.at({1, 2, 1}), WithinAbs(base, 1e-2f));

    // way 2, signal for the forward direction only: only 1 -> 2 is derated
    REQUIRE_THAT(cap.at({2, 1, 2}), WithinAbs(sig, 1e-2f));
    REQUIRE_THAT(cap.at({2, 3, 2}), WithinAbs(base, 1e-2f));
    REQUIRE_THAT(cap.at({2, 2, 3}), WithinAbs(base, 1e-2f));

    // way 3, give_way for the backward direction only: only 3 -> 2 is derated
    REQUIRE_THAT(cap.at({3, 3, 2}), WithinAbs(stop, 1e-2f));
    REQUIRE_THAT(cap.at({3, 1, 2}), WithinAbs(base, 1e-2f));
    REQUIRE_THAT(cap.at({3, 2, 1}), WithinAbs(base, 1e-2f));

    // way 4, stop, no direction: both approaches
    REQUIRE_THAT(cap.at({4, 1, 2}), WithinAbs(stop, 1e-2f));
    REQUIRE_THAT(cap.at({4, 3, 2}), WithinAbs(stop, 1e-2f));
    REQUIRE_THAT(cap.at({4, 2, 3}), WithinAbs(base, 1e-2f));
    std::filesystem::remove(path);
}
