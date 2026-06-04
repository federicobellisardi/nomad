#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <cstdint>

namespace nomad {

// ── Network cleaner ───────────────────────────────────────────────────────────
// Applies topological post-processing to a raw OSM-derived graph:
//
//  1. Isolated component removal
//     Keep the largest weakly-connected component per mode.
//     Removes satellite islands (dead parking lots, private roads, etc.)
//     that would never appear in any routable path.
//
//  2. Degree-2 node simplification
//     Collapse chains of degree-2 nodes (not at signals or intersections)
//     into a single directed edge with concatenated geometry.
//     A typical urban network loses ~30% of its node count, dramatically
//     reducing CH preprocessing time and memory.
//     Signals and intersection nodes are always preserved.
//
//  3. Self-loop removal
//     OSM occasionally contains ways that start and end at the same node.
//
//  4. Duplicate edge removal
//     Multiple OSM ways along the same node pair are merged by keeping the
//     highest-speed / highest-capacity version.
//
// All operations preserve the bidirectional edge pairing invariant
// (forward=2k, reverse=2k+1) by renumbering edges after cleaning.
class NetworkCleaner {
public:
    struct Config {
        uint32_t min_component_size = 5;   // discard components smaller than this
        bool     simplify_topology  = true; // collapse degree-2 chains
        bool     remove_self_loops  = true;
        bool     remove_duplicates  = true;
    };

    struct Report {
        uint32_t nodes_before, nodes_after;
        uint32_t edges_before, edges_after;
        uint32_t components_removed;
        uint32_t nodes_simplified;
        float    largest_component_fraction;  // fraction of original nodes kept
    };

    explicit NetworkCleaner();
    explicit NetworkCleaner(Config cfg);

    // Apply all enabled cleaning steps. Returns a new (cleaned) Graph.
    // The input Graph is consumed (moved from).
    Graph clean(Graph g);

    const Report& last_report() const noexcept { return report_; }

private:
    void remove_small_components(Graph& g);
    void simplify_degree2_nodes (Graph& g);
    void remove_self_loops       (Graph& g);
    void remove_duplicate_edges  (Graph& g);
    void reindex                 (Graph& g);

    Config cfg_;
    mutable Report report_{};
};

} // namespace nomad
