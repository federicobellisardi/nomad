#include <nomad/network/network_cleaner.hpp>

#include <algorithm>
#include <cassert>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>

namespace nomad {

NetworkCleaner::NetworkCleaner() : cfg_(Config{}) {}
NetworkCleaner::NetworkCleaner(Config cfg) : cfg_(cfg) {}

Graph NetworkCleaner::clean(Graph g) {
    report_.nodes_before = g.num_nodes();
    report_.edges_before = g.num_edges();

    if (cfg_.remove_self_loops)   remove_self_loops(g);
    if (cfg_.remove_duplicates)   remove_duplicate_edges(g);
    // simplify_degree2_nodes is O(N²) — disabled until reverse-adjacency is built
    // if (cfg_.simplify_topology)   simplify_degree2_nodes(g);
    remove_small_components(g);
    reindex(g);

    report_.nodes_after = g.num_nodes();
    report_.edges_after = g.num_edges();
    if (report_.nodes_before > 0) {
        report_.largest_component_fraction =
            static_cast<float>(report_.nodes_after) / report_.nodes_before;
    }

    spdlog::info("NetworkCleaner: {} → {} nodes, {} → {} edges",
                  report_.nodes_before, report_.nodes_after,
                  report_.edges_before, report_.edges_after);
    return g;
}

// ── BFS-based connected component analysis ────────────────────────────────────
void NetworkCleaner::remove_small_components(Graph& g) {
    const uint32_t N = g.num_nodes();
    std::vector<int32_t>  component(N, -1);
    std::vector<uint32_t> comp_size;

    uint32_t comp_id = 0;
    for (uint32_t start = 0; start < N; ++start) {
        if (component[start] >= 0) continue;

        // BFS on undirected view (ignoring edge direction)
        std::queue<NodeId> q;
        q.push(start);
        component[start] = comp_id;
        uint32_t sz = 0;

        while (!q.empty()) {
            NodeId u = q.front(); q.pop();
            ++sz;
            for (EdgeId eid : g.out_edges(u)) {
                NodeId v = g.edges[eid].target;
                if (v == kInvalidNode || v >= N) continue; // edge marked invalid
                if (component[v] < 0) {
                    component[v] = comp_id;
                    q.push(v);
                }
            }
        }
        comp_size.push_back(sz);
        ++comp_id;
    }

    // Find the largest component
    uint32_t largest = *std::max_element(comp_size.begin(), comp_size.end());
    uint32_t largest_id = static_cast<uint32_t>(
        std::max_element(comp_size.begin(), comp_size.end()) - comp_size.begin());

    report_.components_removed = 0;
    for (uint32_t c = 0; c < comp_id; ++c) {
        if (c != largest_id && comp_size[c] < cfg_.min_component_size)
            ++report_.components_removed;
    }

    // Mark nodes not in the largest component for removal
    std::vector<bool> keep(N, false);
    for (uint32_t i = 0; i < N; ++i) {
        if (component[i] == static_cast<int32_t>(largest_id) ||
            comp_size[component[i]] >= cfg_.min_component_size) {
            keep[i] = true;
        }
    }

    // Remove edges connecting to removed nodes
    const uint32_t E = g.num_edges();
    for (uint32_t e = 0; e < E; ++e) {
        NodeId t = g.edges[e].target;
        if (t == kInvalidNode || t >= N || !keep[t])
            g.edges[e].target = kInvalidNode;
    }

    // Mark nodes for removal by setting their row_ptr interval to empty
    // (handled by reindex)
    for (uint32_t u = 0; u < N; ++u) {
        if (!keep[u]) {
            g.row_ptr[u + 1] = g.row_ptr[u]; // zero out-degree
        }
    }
}

// ── Self-loop removal ─────────────────────────────────────────────────────────
void NetworkCleaner::remove_self_loops(Graph& g) {
    const uint32_t N = g.num_nodes();
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : g.out_edges(u)) {
            if (g.edges[eid].target == u) {
                g.edges[eid].target = kInvalidNode; // mark for removal
            }
        }
    }
}

// ── Duplicate edge removal ────────────────────────────────────────────────────
void NetworkCleaner::remove_duplicate_edges(Graph& g) {
    const uint32_t N = g.num_nodes();
    for (uint32_t u = 0; u < N; ++u) {
        auto span = g.out_edges(u);
        std::unordered_map<NodeId, EdgeId> seen; // target → first EdgeId
        for (EdgeId eid : span) {
            NodeId v = g.edges[eid].target;
            if (v == kInvalidNode) continue;
            auto [it, inserted] = seen.emplace(v, eid);
            if (!inserted) {
                // Keep the higher-capacity/speed edge
                const EdgeId prev = it->second;
                if (g.edges[eid].capacity > g.edges[prev].capacity) {
                    g.edges[prev].target = kInvalidNode;
                    it->second = eid;
                } else {
                    g.edges[eid].target = kInvalidNode;
                }
            }
        }
    }
}

// ── Degree-2 node simplification ─────────────────────────────────────────────
// A degree-2 node that is NOT a traffic signal and NOT a dead end can be
// contracted: its two edges are merged into one longer edge.
void NetworkCleaner::simplify_degree2_nodes(Graph& g) {
    const uint32_t N = g.num_nodes();

    // Compute in-degree (counting valid edges only)
    std::vector<uint32_t> in_degree(N, 0);
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : g.out_edges(u)) {
            if (g.edges[eid].target != kInvalidNode)
                ++in_degree[g.edges[eid].target];
        }
    }

    uint32_t simplified = 0;
    // Iterate until no more degree-2 nodes can be contracted
    bool changed = true;
    while (changed) {
        changed = false;
        for (uint32_t u = 0; u < N; ++u) {
            // Skip signal nodes and already-removed nodes
            const auto& nd = g.nodes[u];
            if (nd.intersection_type != static_cast<uint8_t>(IntersectionType::Simple))
                continue;

            // Count valid out-edges
            uint32_t out = 0;
            EdgeId out_edge = kInvalidEdge;
            for (EdgeId eid : g.out_edges(u)) {
                if (g.edges[eid].target != kInvalidNode) {
                    ++out;
                    out_edge = eid;
                }
            }

            if (out != 1 || in_degree[u] != 1) continue;

            // u has exactly 1 in-edge and 1 out-edge → can be contracted
            // Find the predecessor node
            NodeId succ = g.edges[out_edge].target;
            if (succ == u) continue; // self-loop guard

            // Find predecessor of u (the node with an edge to u)
            // This requires scanning all predecessors — O(N) worst case.
            // For production, maintain an explicit reverse adjacency.
            // Here we do a simple scan for correctness.
            NodeId pred = kInvalidNode;
            EdgeId pred_edge = kInvalidEdge;
            for (uint32_t p = 0; p < N; ++p) {
                for (EdgeId eid : g.out_edges(p)) {
                    if (g.edges[eid].target == u && eid != out_edge) {
                        pred = p;
                        pred_edge = eid;
                        break;
                    }
                }
                if (pred != kInvalidNode) break;
            }

            if (pred == kInvalidNode || pred == succ) continue;

            // Merge: extend pred→u edge to pred→succ
            g.edges[pred_edge].target   = succ;
            g.edges[pred_edge].length_m += g.edges[out_edge].length_m;
            // Take minimum speed (conservative)
            g.edges[pred_edge].free_flow_speed =
                std::min(g.edges[pred_edge].free_flow_speed,
                          g.edges[out_edge].free_flow_speed);
            // Concatenate geometry
            // (omitted for brevity; production would merge geom_coords here)

            // Invalidate u's out-edge
            g.edges[out_edge].target = kInvalidNode;
            in_degree[succ] -= 1; // u's out-edge removed
            // u now has 0 valid out-edges; it will be removed by reindex

            ++simplified;
            changed = true;
        }
    }

    report_.nodes_simplified = simplified;
}

// ── Reindex: compact the graph after edge/node removals ───────────────────────
void NetworkCleaner::reindex(Graph& g) {
    const uint32_t N_old = g.num_nodes();
    const uint32_t E_old = g.num_edges();

    // Build new NodeId mapping (skip nodes with no valid edges in or out)
    std::vector<uint32_t> node_map(N_old, kInvalidNode);
    uint32_t new_N = 0;

    // Mark nodes that have at least one valid outgoing edge or are reachable
    for (uint32_t u = 0; u < N_old; ++u) {
        for (EdgeId eid : g.out_edges(u)) {
            if (g.edges[eid].target != kInvalidNode) {
                node_map[u] = 0; // mark as used (will assign id below)
                node_map[g.edges[eid].target] = 0;
            }
        }
    }
    for (uint32_t u = 0; u < N_old; ++u) {
        if (node_map[u] == 0) node_map[u] = new_N++;
        else node_map[u] = kInvalidNode; // truly isolated
    }

    // Build new edge list
    std::vector<EdgeData> new_edges;
    std::vector<EdgeId>   new_col;
    new_edges.reserve(E_old);
    new_col.reserve(E_old);

    std::vector<uint32_t> new_row_ptr(new_N + 1, 0);
    // First pass: count
    for (uint32_t u = 0; u < N_old; ++u) {
        if (node_map[u] == kInvalidNode) continue;
        uint32_t nu = node_map[u];
        for (EdgeId eid : g.out_edges(u)) {
            if (g.edges[eid].target == kInvalidNode) continue;
            if (node_map[g.edges[eid].target] == kInvalidNode) continue;
            ++new_row_ptr[nu + 1];
        }
    }
    // Prefix sum
    for (uint32_t i = 0; i < new_N; ++i)
        new_row_ptr[i + 1] += new_row_ptr[i];

    uint32_t new_E = new_row_ptr[new_N];
    new_edges.resize(new_E);
    new_col.resize(new_E);

    // Second pass: fill
    std::vector<uint32_t> fill = new_row_ptr;
    uint32_t new_eid = 0;
    for (uint32_t u = 0; u < N_old; ++u) {
        if (node_map[u] == kInvalidNode) continue;
        uint32_t nu = node_map[u];
        for (EdgeId eid : g.out_edges(u)) {
            if (g.edges[eid].target == kInvalidNode) continue;
            NodeId nv = node_map[g.edges[eid].target];
            if (nv == kInvalidNode) continue;

            EdgeData ed         = g.edges[eid];
            ed.target           = nv;
            uint32_t pos        = fill[nu]++;
            new_edges[pos]      = ed;
            new_col[pos]        = new_eid++;
        }
    }

    // Rebuild node array
    std::vector<NodeData> new_nodes(new_N);
    for (uint32_t u = 0; u < N_old; ++u) {
        if (node_map[u] != kInvalidNode)
            new_nodes[node_map[u]] = g.nodes[u];
    }

    g.row_ptr = std::move(new_row_ptr);
    g.col_idx = std::move(new_col);
    g.edges   = std::move(new_edges);
    g.nodes   = std::move(new_nodes);
    // Geometry sidecar rebuild omitted (kept as-is in this phase)
    g.geom_ptr.assign(new_E + 1, 0);
}

} // namespace nomad
