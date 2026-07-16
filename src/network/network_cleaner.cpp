#include <nomad/network/network_cleaner.hpp>

#include <algorithm>
#include <cassert>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>

#include <spdlog/spdlog.h>

namespace nomad {

NetworkCleaner::NetworkCleaner() : cfg_(Config{}) {}
NetworkCleaner::NetworkCleaner(Config cfg) : cfg_(cfg) {}

Graph NetworkCleaner::clean(Graph g) {
    report_.nodes_before = g.num_nodes();
    report_.edges_before = g.num_edges();

    if (cfg_.remove_self_loops)   remove_self_loops(g);
    if (cfg_.remove_duplicates)   remove_duplicate_edges(g);
    if (cfg_.simplify_topology)   simplify_degree2_nodes(g);
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

    // Mark edges TO removed nodes
    const uint32_t E = g.num_edges();
    for (uint32_t e = 0; e < E; ++e) {
        NodeId t = g.edges[e].target;
        if (t == kInvalidNode || t >= N || !keep[t])
            g.edges[e].target = kInvalidNode;
    }
    // Mark edges FROM removed nodes.
    // Previously this was done by zeroing row_ptr[u+1], but that corrupts the CSR
    // layout for the immediately following kept node: it would inherit the removed
    // node's edge slots in reindex(), assigning them a wrong source node while
    // keeping the original length_m — producing edges whose endpoints are many km
    // apart but whose length_m is only a few metres.
    for (uint32_t u = 0; u < N; ++u) {
        if (!keep[u]) {
            for (EdgeId eid : g.out_edges(u))
                g.edges[eid].target = kInvalidNode;
        }
    }
    // reindex() compacts the graph; no row_ptr surgery needed here.
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
// Contracts pass-through nodes that connect exactly two distinct undirected
// neighbours. Handles both one-way (in=1, out=1) and bidirectional (in=2,
// out=2 with 2 distinct nodes) segments. Runs in O(N+E) using precomputed
// reverse adjacency and a worklist; no O(N²) predecessor scan.
//
// Also collapses Simple nodes whose shortest incident edge is below
// cfg_.min_edge_length_m even if they nominally have higher directed degree
// (handles split-node artefacts at complex intersections).
void NetworkCleaner::simplify_degree2_nodes(Graph& g) {
    const uint32_t N = g.num_nodes();
    const uint32_t E = g.num_edges();

    // ── Pre-compute edge→source mapping ──────────────────────────────────────
    std::vector<NodeId> edge_src(E, kInvalidNode);
    for (uint32_t u = 0; u < N; ++u)
        for (EdgeId eid : g.out_edges(u))
            edge_src[eid] = u;

    // ── Pre-compute per-node in-edge lists ───────────────────────────────────
    // Entries become stale as targets are redirected; always filter by
    // checking g.edges[eid].target == u before using.
    std::vector<std::vector<EdgeId>> in_edges(N);
    in_edges.reserve(N);
    for (uint32_t e = 0; e < E; ++e) {
        NodeId v = g.edges[e].target;
        if (v != kInvalidNode && v < N)
            in_edges[v].push_back(static_cast<EdgeId>(e));
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    // Valid in-edges to u: filter stale entries (target may have been redirected).
    auto valid_in = [&](NodeId u, std::vector<EdgeId>& out) {
        out.clear();
        for (EdgeId eid : in_edges[u])
            if (g.edges[eid].target == u)
                out.push_back(eid);
    };

    // Valid out-edges from u (non-invalidated, non-self-loop).
    auto valid_out = [&](NodeId u, std::vector<EdgeId>& out) {
        out.clear();
        for (EdgeId eid : g.out_edges(u)) {
            NodeId v = g.edges[eid].target;
            if (v != kInvalidNode && v != u)
                out.push_back(eid);
        }
    };

    // Distinct undirected neighbours of u (sources of in-edges + targets of
    // out-edges, excluding u itself).
    auto undir_nb = [&](NodeId u, std::vector<EdgeId>& ins,
                         std::vector<EdgeId>& outs,
                         std::vector<NodeId>& nb) {
        valid_in(u, ins);
        valid_out(u, outs);
        nb.clear();
        for (EdgeId e : ins)  { NodeId s = edge_src[e]; if (s != kInvalidNode && s != u) nb.push_back(s); }
        for (EdgeId e : outs) nb.push_back(g.edges[e].target);
        std::sort(nb.begin(), nb.end());
        nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
    };

    // Is node u eligible for contraction?
    // Criterion A: exactly 2 undirected neighbours and Simple type, with a
    //              clean one-to-one redirect mapping between in- and
    //              out-edges (see is_valid_pass_through below).
    // Criterion B: Simple type and at least one incident edge < min_edge_length_m
    //              and exactly 2 undirected neighbours (regardless of directed degree).
    auto contractable = [&](NodeId u,
                             std::vector<EdgeId>& ins,
                             std::vector<EdgeId>& outs,
                             std::vector<NodeId>& nb) -> bool {
        if (g.nodes[u].intersection_type != static_cast<uint8_t>(IntersectionType::Simple))
            return false;
        undir_nb(u, ins, outs, nb);
        if (nb.size() != 2) return false;

        // A valid pass-through redirects every in-edge from neighbour X to an
        // out-edge toward the *other* neighbour Y. This must hold for BOTH
        // one-way chains (A→u→B: 1 in-edge from A, 1 out-edge to B — the
        // common case for motorways/trunk roads, which OSM usually maps as
        // separate one-way carriageways with no reverse edge at all) and
        // two-way chains (each neighbour contributes both an in- and an
        // out-edge). Checking "n_in==1 && n_out==1 per neighbour" as the sole
        // criterion — as a naive reading of "degree-2" suggests — silently
        // rejects every one-way pass-through, since a one-way neighbour only
        // ever supplies an in-edge *or* an out-edge, never both.
        NodeId A = nb[0];
        uint32_t in_from_A = 0, in_from_B = 0, out_to_A = 0, out_to_B = 0;
        for (EdgeId e : ins)  (edge_src[e] == A ? in_from_A : in_from_B)++;
        for (EdgeId e : outs) (g.edges[e].target == A ? out_to_A : out_to_B)++;
        bool is_strict_degree2 = (in_from_A == out_to_B) && (in_from_B == out_to_A);

        if (is_strict_degree2) {
            // Criterion A: pass-through node on a straight road segment — always contract.
        } else if (cfg_.min_edge_length_m > 0.0f) {
            // Criterion B: not strict degree-2, but at least one incident edge is a
            // micro-segment (<min_edge_length_m). Contract to clean up OSM split artefacts.
            bool has_short = false;
            for (EdgeId e : ins)
                if (g.edges[e].length_m < cfg_.min_edge_length_m) { has_short = true; break; }
            if (!has_short)
                for (EdgeId e : outs)
                    if (g.edges[e].length_m < cfg_.min_edge_length_m) { has_short = true; break; }
            if (!has_short) return false;
        } else {
            return false; // strict mode: skip non-strict nodes
        }
        return true;
    };

    // ── Initialise worklist ───────────────────────────────────────────────────
    std::queue<NodeId> wl;
    std::vector<bool>  in_wl(N, false);
    std::vector<EdgeId> tmp_ins, tmp_outs;
    std::vector<NodeId> tmp_nb;

    for (uint32_t u = 0; u < N; ++u) {
        if (contractable(u, tmp_ins, tmp_outs, tmp_nb)) {
            wl.push(u);
            in_wl[u] = true;
        }
    }

    uint32_t contracted = 0;

    // ── Worklist processing ───────────────────────────────────────────────────
    while (!wl.empty()) {
        NodeId u = wl.front(); wl.pop();
        in_wl[u] = false;

        if (!contractable(u, tmp_ins, tmp_outs, tmp_nb)) continue;

        std::vector<EdgeId>& ins  = tmp_ins;
        std::vector<EdgeId>& outs = tmp_outs;
        std::vector<NodeId>& nb   = tmp_nb;

        NodeId A = nb[0], B = nb[1];

        // For each in-edge (X→u), find the pass-through out-edge (u→Y, Y≠X).
        // Redirect X→u to X→Y and invalidate u→Y.
        for (EdgeId in_e : ins) {
            NodeId X = edge_src[in_e];
            if (X == kInvalidNode) continue;
            NodeId Y = (X == A) ? B : A;  // the other neighbour

            // Find u→Y
            EdgeId out_e = kInvalidEdge;
            for (EdgeId oe : outs) {
                if (g.edges[oe].target == Y) { out_e = oe; break; }
            }
            if (out_e == kInvalidEdge) continue;

            // Merge: X→u→Y becomes X→Y
            g.edges[in_e].target         = Y;
            g.edges[in_e].length_m      += g.edges[out_e].length_m;
            g.edges[in_e].free_flow_speed = std::min(g.edges[in_e].free_flow_speed,
                                                      g.edges[out_e].free_flow_speed);
            g.edges[in_e].capacity        = std::min(g.edges[in_e].capacity,
                                                      g.edges[out_e].capacity);

            // Track the redirected edge so Y knows about it
            in_edges[Y].push_back(in_e);

            // Invalidate the absorbed out-edge
            g.edges[out_e].target = kInvalidNode;
        }

        ++contracted;

        // Neighbours may now satisfy the degree-2 criterion
        for (NodeId nb_node : {A, B}) {
            if (nb_node == kInvalidNode || in_wl[nb_node]) continue;
            if (contractable(nb_node, tmp_ins, tmp_outs, tmp_nb)) {
                wl.push(nb_node);
                in_wl[nb_node] = true;
            }
        }
    }

    report_.nodes_simplified = contracted;
    spdlog::info("NetworkCleaner: simplified {} degree-2 nodes (min_edge_length_m={})",
                 contracted, cfg_.min_edge_length_m);
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
