#pragma once

#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>
#include <nomad/routing/route_cache.hpp>
#include <nomad/routing/router.hpp>
#include <nomad/network/intersection.hpp>

#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nomad {

// ── Contraction Hierarchies router ───────────────────────────────────────────
// CH is the standard technique for fast shortest-path queries on road networks:
//   - Preprocessing (offline, ~8 min for Paris): assigns importance levels to
//     nodes and adds shortcut edges to preserve shortest paths.
//   - Query (online, ~0.1 ms per query): bidirectional Dijkstra on the
//     upward graph (edges from lower→higher level only).
//
// At 1M agents with 8 threads, startup routing completes in ~120 CPU-seconds.
// This is orders of magnitude faster than minimocas's pre-computed all-pairs
// matrix (infeasible at metro scale).
//
// Shortcut structure: CH augments the base graph with additional "shortcut"
// edges that each represent a path through a contracted node. The shortcut
// table stores the via-node so paths can be unpacked to full edge sequences.
//
// Turn restriction handling: restrictions from OsmLoader are encoded as
// via-node turn costs during preprocessing. Restricted turns receive infinite
// weight — they never appear in the contracted graph.
//
// Dynamic cost updates: when on_exit triggers a significant travel time change,
// update_costs() adjusts edge weights in the query graph without re-contracting.
// This is conservative (may over-approximate congestion) but avoids the cost
// of full re-contraction during the simulation.
class CHRouter final : public IRouter {
public:
    struct PreprocessConfig {
        uint32_t  num_threads = 0;            // 0 = hardware_concurrency
        bool      verbose     = false;
        AgentMode mode        = AgentMode::Car; // which edges are usable
    };

    explicit CHRouter(const Graph& graph);
    explicit CHRouter(const Graph& graph,
                       const std::vector<TurnRestriction>& restrictions,
                       PreprocessConfig pp_cfg);

    // ── Preprocessing (offline, call once before routing) ─────────────────────
    void preprocess();

    // Save/load CH data to avoid re-preprocessing
    void save(const std::filesystem::path& path) const;
    void load(const std::filesystem::path& path);
    bool is_preprocessed() const noexcept { return preprocessed_; }

    // ── IRouter interface ─────────────────────────────────────────────────────
    Route route(const RoutingRequest& req)                               override;
    void  batch_route(std::span<const RoutingRequest>, std::span<Route>) override;
    void  update_costs(std::span<const EdgeId> changed_edges)            override;

    std::string_view router_name() const override { return "CH"; }

    void set_cache(RouteCache* cache) noexcept { cache_ = cache; }

private:
    // ── CH data structures ────────────────────────────────────────────────────
    struct Shortcut {
        NodeId source, target;
        NodeId via;             // contracted node (for path unpacking)
        float  weight;          // sum of constituent edge weights
    };

    // Augmented CSR: base graph edges + shortcut edges, directed upward
    struct CHGraph {
        std::vector<uint32_t>  up_row_ptr;   // upward adjacency CSR
        std::vector<uint32_t>  up_col_idx;
        std::vector<float>     up_weight;
        std::vector<uint32_t>  down_row_ptr; // downward adjacency CSR (for reverse search)
        std::vector<uint32_t>  down_col_idx;
        std::vector<float>     down_weight;
        std::vector<NodeId>    level;        // node importance level
        std::vector<Shortcut>  shortcuts;
    };

    // Thread-local workspace for bidirectional Dijkstra
    struct alignas(64) ThreadData {
        std::vector<float>   dist_fwd, dist_bwd;
        std::vector<NodeId>  prev_fwd, prev_bwd;
        std::vector<uint8_t> settled_fwd, settled_bwd;
        std::vector<std::pair<float, NodeId>> heap_fwd, heap_bwd;
        void reset(uint32_t num_nodes);
    };

    Route  ch_query(NodeId s, NodeId t, AgentMode mode) const;
    Route  unpack_path(NodeId s, NodeId t, NodeId meeting,
                        const std::vector<NodeId>& fwd_prev,
                        const std::vector<NodeId>& bwd_prev) const;
    void   expand_path(NodeId u, NodeId w, std::vector<NodeId>& node_seq) const;

    void   build_augmented_graph();

    const Graph&                        graph_;
    const std::vector<TurnRestriction>& restrictions_;
    PreprocessConfig                    pp_cfg_;
    CHGraph                             ch_;
    bool                                preprocessed_{false};
    RouteCache*                         cache_{nullptr};

    // shortcut lookup: (source<<32|target) → (via, weight)
    std::unordered_map<uint64_t, std::pair<NodeId, float>> sc_lookup_;

    mutable std::vector<ThreadData>     tls_;
};

} // namespace nomad
