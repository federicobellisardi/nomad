#include <nomad/routing/ch_router.hpp>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_arena.h>
#include <spdlog/spdlog.h>

namespace nomad {

static constexpr float    kInf         = std::numeric_limits<float>::infinity();
static constexpr NodeId   kNoNode      = std::numeric_limits<NodeId>::max();
static constexpr uint32_t kNoShortcut  = std::numeric_limits<uint32_t>::max();

static const std::vector<TurnRestriction> kEmptyRestrictions;

// ── Constructors ──────────────────────────────────────────────────────────────

CHRouter::CHRouter(const Graph& graph)
    : CHRouter(graph, kEmptyRestrictions, PreprocessConfig{}) {}

CHRouter::CHRouter(const Graph& graph,
                    const std::vector<TurnRestriction>& restrictions,
                    PreprocessConfig pp_cfg)
    : graph_(graph), restrictions_(restrictions), pp_cfg_(pp_cfg)
{
    if (pp_cfg_.num_threads == 0)
        pp_cfg_.num_threads = std::thread::hardware_concurrency();
    tls_.resize(std::max(1u, pp_cfg_.num_threads));
}

void CHRouter::ThreadData::reset(uint32_t N) {
    dist_fwd   .assign(N, kInf);
    dist_bwd   .assign(N, kInf);
    prev_fwd   .assign(N, kNoNode);
    prev_bwd   .assign(N, kNoNode);
    settled_fwd.assign(N, 0);
    settled_bwd.assign(N, 0);
    heap_fwd.clear();
    heap_bwd.clear();
}

// ── CH Preprocessing ──────────────────────────────────────────────────────────
//
// Algorithm: Geisberger et al. (2008) "Contraction Hierarchies: Faster and
// Simpler Hierarchical Routing in Road Networks".
//
// Working graph: starts as the base graph, grows as shortcuts are added.
// Nodes are contracted in ascending edge-difference order (lazy updates).
// Witness search: limited Dijkstra (max_hops = 5) to prune unnecessary shortcuts.

// Edge in the working graph during preprocessing
struct WorkEdge {
    NodeId  target;
    float   cost;
    uint32_t sc_idx;    // kNoShortcut = base edge, else index into shortcuts_
};

using WAdjList = std::vector<std::vector<WorkEdge>>;

// Compute edge difference for node v: shortcuts_needed - edges_removed
static int compute_edge_diff(NodeId v,
                              const WAdjList& out_adj,
                              const WAdjList& in_adj,
                              const std::vector<bool>& contracted) {
    const auto& preds = in_adj[v];
    const auto& succs = out_adj[v];

    float max_cost = 0;
    for (const auto& s : succs) max_cost = std::max(max_cost, s.cost);

    int shortcuts_needed = 0;
    int edges_removed    = 0;

    for (const auto& [u, uv, _pu] : preds) {
        if (contracted[u]) continue;
        ++edges_removed;

        // Witness search from u avoiding v
        std::unordered_map<NodeId, float> dist_u;
        {
            using PQ = std::pair<float, NodeId>;
            std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
            dist_u[u] = 0.0f;
            pq.push({0.0f, u});
            int pops = 0;
            while (!pq.empty() && pops < 200) {
                auto [d, node] = pq.top(); pq.pop();
                if (d > dist_u[node] + 1e-6f) continue;
                if (node == v) continue; // skip v
                ++pops;
                for (const auto& e : out_adj[node]) {
                    if (e.target == v) continue;
                    if (contracted[e.target]) continue;
                    float nd = d + e.cost;
                    if (nd > uv + max_cost) continue;
                    auto& dd = dist_u[e.target];
                    if (nd < dd) { dd = nd; pq.push({nd, e.target}); }
                }
            }
        }

        for (const auto& [w, vw, _pw] : succs) {
            if (contracted[w] || w == u) continue;
            float sc_cost = uv + vw;
            auto it = dist_u.find(w);
            if (it == dist_u.end() || it->second > sc_cost - 1e-6f)
                ++shortcuts_needed;
        }
    }

    for (const auto& s : succs)
        if (!contracted[s.target]) ++edges_removed;

    return shortcuts_needed - edges_removed;
}

void CHRouter::preprocess() {
    const uint32_t N = graph_.num_nodes();

    spdlog::info("CHRouter: preprocessing {} nodes, {} edges...",
                  N, graph_.num_edges());

    // ── Build initial working graph ───────────────────────────────────────────
    WAdjList out_adj(N), in_adj(N);
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : graph_.out_edges(u)) {
            const auto& e = graph_.edges[eid];
            if (e.target == kInvalidNode) continue;
            if (!road_class_accessible(static_cast<RoadClass>(e.road_class), pp_cfg_.mode))
                continue;
            float cost = e.free_flow_speed > 0
                ? e.length_m / e.free_flow_speed : kInf;
            if (cost >= kInf) continue;
            // De-duplicate: keep minimum cost parallel edge
            bool found = false;
            for (auto& we : out_adj[u]) {
                if (we.target == e.target) { we.cost = std::min(we.cost, cost); found = true; break; }
            }
            if (!found) {
                out_adj[u].push_back({e.target, cost, kNoShortcut});
                in_adj[e.target].push_back({u, cost, kNoShortcut});
            }
        }
    }

    ch_.level.assign(N, 0);
    ch_.shortcuts.clear();

    std::vector<bool> contracted(N, false);
    uint32_t level = 0;

    // ── Priority queue: (edge_diff, node) ─────────────────────────────────────
    using PQ = std::pair<int, NodeId>;
    std::priority_queue<PQ, std::vector<PQ>, std::greater<PQ>> pq;
    std::vector<int> priority(N, 0);
    std::vector<bool> in_pq(N, false);

    // Initial priorities (quick estimate: in_deg * out_deg)
    for (uint32_t v = 0; v < N; ++v) {
        int ind = static_cast<int>(in_adj[v].size());
        int outd = static_cast<int>(out_adj[v].size());
        priority[v] = ind * outd - (ind + outd);
        pq.push({priority[v], v});
        in_pq[v] = true;
    }

    // ── Main contraction loop ─────────────────────────────────────────────────
    uint32_t contracted_count = 0;
    uint32_t report_step      = N / 20; // log every 5%

    while (!pq.empty()) {
        auto [p, v] = pq.top(); pq.pop();
        if (contracted[v]) continue;

        // Lazy update: recompute exact priority
        int exact = compute_edge_diff(v, out_adj, in_adj, contracted);
        if (exact > p) {
            priority[v] = exact;
            pq.push({exact, v});
            continue;
        }

        // ── Contract v ───────────────────────────────────────────────────────
        ch_.level[v] = level++;
        contracted[v] = true;
        ++contracted_count;

        if (report_step > 0 && contracted_count % report_step == 0)
            spdlog::info("  CH: contracted {}/{} nodes, {} shortcuts so far",
                          contracted_count, N, ch_.shortcuts.size());

        // Determine max outgoing cost from v
        float max_vw = 0;
        for (const auto& s : out_adj[v])
            if (!contracted[s.target]) max_vw = std::max(max_vw, s.cost);

        for (const auto& [u, uv_cost, _pu] : in_adj[v]) {
            if (contracted[u]) continue;

            float search_limit = uv_cost + max_vw;

            // Witness search from u avoiding v
            std::unordered_map<NodeId, float> dist_u;
            dist_u[u] = 0.0f;
            {
                using PQ2 = std::pair<float, NodeId>;
                std::priority_queue<PQ2, std::vector<PQ2>, std::greater<PQ2>> spq;
                spq.push({0.0f, u});
                int pops = 0;
                while (!spq.empty() && pops < 500) {
                    auto [d, node] = spq.top(); spq.pop();
                    if (d > dist_u[node] + 1e-6f) continue;
                    if (node == v) continue;
                    ++pops;
                    for (const auto& we : out_adj[node]) {
                        if (we.target == v) continue;
                        if (contracted[we.target]) continue;
                        float nd = d + we.cost;
                        if (nd > search_limit + 1e-6f) continue;
                        auto& dd = dist_u[we.target];
                        if (nd < dd) { dd = nd; spq.push({nd, we.target}); }
                    }
                }
            }

            for (const auto& [w, vw_cost, _pw] : out_adj[v]) {
                if (contracted[w] || w == u) continue;
                float sc_cost = uv_cost + vw_cost;
                auto it = dist_u.find(w);
                bool need_sc = (it == dist_u.end() || it->second > sc_cost - 1e-6f);
                if (!need_sc) continue;

                uint32_t sc_idx = static_cast<uint32_t>(ch_.shortcuts.size());
                ch_.shortcuts.push_back({u, w, v, sc_cost});

                // Update working graph
                bool found = false;
                for (auto& we : out_adj[u]) {
                    if (we.target == w) {
                        if (sc_cost < we.cost) { we.cost = sc_cost; we.sc_idx = sc_idx; }
                        found = true; break;
                    }
                }
                if (!found) {
                    out_adj[u].push_back({w, sc_cost, sc_idx});
                    in_adj[w].push_back({u, sc_cost, sc_idx});
                } else {
                    for (auto& we : in_adj[w]) {
                        if (we.target == u) { we.cost = std::min(we.cost, sc_cost); break; }
                    }
                }
            }
        }

        // Remove v from adjacency of its neighbors
        for (const auto& [u, _, _pu] : in_adj[v]) {
            auto& ov = out_adj[u];
            ov.erase(std::remove_if(ov.begin(), ov.end(),
                [v](const WorkEdge& e){ return e.target == v; }), ov.end());
        }
        for (const auto& [w, _, _pw] : out_adj[v]) {
            auto& iv = in_adj[w];
            iv.erase(std::remove_if(iv.begin(), iv.end(),
                [v](const WorkEdge& e){ return e.target == v; }), iv.end());
        }
        out_adj[v].clear();
        in_adj[v].clear();
    }

    preprocessed_ = true;
    spdlog::info("CHRouter: preprocessing complete. {} shortcuts, {} levels",
                  ch_.shortcuts.size(), level);

    build_augmented_graph();
}

// ── Build overlay CSR graph ───────────────────────────────────────────────────

void CHRouter::build_augmented_graph() {
    const uint32_t N = graph_.num_nodes();

    // Count upward/downward edges
    ch_.up_row_ptr  .assign(N + 1, 0);
    ch_.down_row_ptr.assign(N + 1, 0);

    // Base graph edges
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : graph_.out_edges(u)) {
            const auto& ed = graph_.edges[eid];
            if (ed.target == kInvalidNode) continue;
            if (!road_class_accessible(static_cast<RoadClass>(ed.road_class), pp_cfg_.mode))
                continue;
            float cost = ed.free_flow_speed > 0
                ? ed.length_m / ed.free_flow_speed : kInf;
            if (cost >= kInf) continue;
            NodeId v = ed.target;
            if (ch_.level[v] > ch_.level[u])       ++ch_.up_row_ptr  [u + 1];
            else if (ch_.level[v] < ch_.level[u])  ++ch_.down_row_ptr[v + 1];
        }
    }
    // Shortcuts
    for (const auto& sc : ch_.shortcuts) {
        if (ch_.level[sc.target] > ch_.level[sc.source]) ++ch_.up_row_ptr  [sc.source + 1];
        else                                               ++ch_.down_row_ptr[sc.target + 1];
    }

    // Prefix sum
    for (uint32_t i = 0; i < N; ++i) {
        ch_.up_row_ptr  [i + 1] += ch_.up_row_ptr  [i];
        ch_.down_row_ptr[i + 1] += ch_.down_row_ptr[i];
    }

    uint32_t tot_up   = ch_.up_row_ptr  [N];
    uint32_t tot_down = ch_.down_row_ptr[N];
    ch_.up_col_idx  .resize(tot_up);
    ch_.up_weight   .resize(tot_up);
    ch_.down_col_idx.resize(tot_down);
    ch_.down_weight .resize(tot_down);

    std::vector<uint32_t> up_pos  (N, 0);
    std::vector<uint32_t> down_pos(N, 0);

    // Fill base edges
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : graph_.out_edges(u)) {
            const auto& ed = graph_.edges[eid];
            if (ed.target == kInvalidNode) continue;
            if (!road_class_accessible(static_cast<RoadClass>(ed.road_class), pp_cfg_.mode))
                continue;
            float cost = ed.free_flow_speed > 0
                ? ed.length_m / ed.free_flow_speed : kInf;
            if (cost >= kInf) continue;
            NodeId v = ed.target;
            if (ch_.level[v] > ch_.level[u]) {
                uint32_t pos = ch_.up_row_ptr[u] + up_pos[u]++;
                ch_.up_col_idx[pos] = v;
                ch_.up_weight [pos] = cost;
            } else if (ch_.level[v] < ch_.level[u]) {
                uint32_t pos = ch_.down_row_ptr[v] + down_pos[v]++;
                ch_.down_col_idx[pos] = u;
                ch_.down_weight [pos] = cost;
            }
        }
    }
    // Fill shortcuts
    for (const auto& sc : ch_.shortcuts) {
        if (ch_.level[sc.target] > ch_.level[sc.source]) {
            uint32_t pos = ch_.up_row_ptr[sc.source] + up_pos[sc.source]++;
            ch_.up_col_idx[pos] = sc.target;
            ch_.up_weight [pos] = sc.weight;
        } else {
            uint32_t pos = ch_.down_row_ptr[sc.target] + down_pos[sc.target]++;
            ch_.down_col_idx[pos] = sc.source;
            ch_.down_weight [pos] = sc.weight;
        }
    }

    // Build shortcut lookup for path unpacking: (source, target) → via
    sc_lookup_.clear();
    sc_lookup_.reserve(ch_.shortcuts.size());
    for (const auto& sc : ch_.shortcuts) {
        uint64_t key = (static_cast<uint64_t>(sc.source) << 32) | sc.target;
        // Keep minimum-cost entry
        auto it = sc_lookup_.find(key);
        if (it == sc_lookup_.end() || sc.weight < it->second.second)
            sc_lookup_[key] = {sc.via, sc.weight};
    }

    spdlog::info("CHRouter: overlay graph: {} up-edges, {} down-edges",
                  tot_up, tot_down);
}

// ── CH Query ──────────────────────────────────────────────────────────────────

Route CHRouter::ch_query(NodeId s, NodeId t, AgentMode /*mode*/) const {
    if (!preprocessed_) {
        spdlog::warn("CHRouter: not preprocessed");
        return {};
    }
    if (s == t) { Route r; r.is_valid = true; return r; }

    const uint32_t N = graph_.num_nodes();
    int slot = tbb::this_task_arena::current_thread_index();
    if (slot < 0 || slot >= static_cast<int>(tls_.size())) slot = 0;
    ThreadData& td = const_cast<ThreadData&>(tls_[slot]);
    td.reset(N);

    using PQ = std::pair<float, NodeId>;
    auto cmp = std::greater<PQ>{};

    td.dist_fwd[s] = 0.0f;
    td.dist_bwd[t] = 0.0f;
    td.heap_fwd.push_back({0.0f, s});
    td.heap_bwd.push_back({0.0f, t});
    std::make_heap(td.heap_fwd.begin(), td.heap_fwd.end(), cmp);
    std::make_heap(td.heap_bwd.begin(), td.heap_bwd.end(), cmp);

    float  best    = kInf;
    NodeId meeting = kNoNode;

    while (!td.heap_fwd.empty() || !td.heap_bwd.empty()) {

        // ── Forward step (upward edges) ───────────────────────────────────────
        if (!td.heap_fwd.empty()) {
            std::pop_heap(td.heap_fwd.begin(), td.heap_fwd.end(), cmp);
            auto [d, u] = td.heap_fwd.back(); td.heap_fwd.pop_back();
            if (!td.settled_fwd[u]) {
                td.settled_fwd[u] = 1;
                if (d < best) {
                    if (td.settled_bwd[u]) {
                        float total = d + td.dist_bwd[u];
                        if (total < best) { best = total; meeting = u; }
                    }
                    for (uint32_t i = ch_.up_row_ptr[u]; i < ch_.up_row_ptr[u + 1]; ++i) {
                        NodeId v = ch_.up_col_idx[i];
                        float  w = ch_.up_weight [i];
                        float nd = d + w;
                        if (nd < td.dist_fwd[v]) {
                            td.dist_fwd[v] = nd;
                            td.prev_fwd[v] = u;
                            td.heap_fwd.push_back({nd, v});
                            std::push_heap(td.heap_fwd.begin(), td.heap_fwd.end(), cmp);
                        }
                    }
                }
            }
        }

        // ── Backward step (downward edges reversed) ───────────────────────────
        if (!td.heap_bwd.empty()) {
            std::pop_heap(td.heap_bwd.begin(), td.heap_bwd.end(), cmp);
            auto [d, u] = td.heap_bwd.back(); td.heap_bwd.pop_back();
            if (!td.settled_bwd[u]) {
                td.settled_bwd[u] = 1;
                if (d < best) {
                    if (td.settled_fwd[u]) {
                        float total = td.dist_fwd[u] + d;
                        if (total < best) { best = total; meeting = u; }
                    }
                    // down_row_ptr[u] = edges INTO u from higher-level nodes
                    // in backward search we traverse these in reverse
                    for (uint32_t i = ch_.down_row_ptr[u]; i < ch_.down_row_ptr[u + 1]; ++i) {
                        NodeId v = ch_.down_col_idx[i]; // v has higher level than u
                        float  w = ch_.down_weight [i];
                        float nd = d + w;
                        if (nd < td.dist_bwd[v]) {
                            td.dist_bwd[v] = nd;
                            td.prev_bwd[v] = u;
                            td.heap_bwd.push_back({nd, v});
                            std::push_heap(td.heap_bwd.begin(), td.heap_bwd.end(), cmp);
                        }
                    }
                }
            }
        }
    }

    if (meeting == kNoNode || best >= kInf) return {};
    return unpack_path(s, t, meeting, td.prev_fwd, td.prev_bwd);
}

// ── Path unpacking ────────────────────────────────────────────────────────────

void CHRouter::expand_path(NodeId u, NodeId w,
                            std::vector<NodeId>& node_seq) const {
    uint64_t key = (static_cast<uint64_t>(u) << 32) | w;
    auto it = sc_lookup_.find(key);
    if (it == sc_lookup_.end()) {
        // Base edge
        node_seq.push_back(w);
        return;
    }
    NodeId via = it->second.first;
    expand_path(u, via, node_seq);
    expand_path(via, w, node_seq);
}

Route CHRouter::unpack_path(NodeId s, NodeId t, NodeId meeting,
                              const std::vector<NodeId>& fwd_prev,
                              const std::vector<NodeId>& bwd_prev) const {
    // Forward path: s → meeting
    std::vector<NodeId> fwd_nodes;
    fwd_nodes.push_back(s);
    {
        std::vector<NodeId> rev;
        NodeId cur = meeting;
        while (cur != s && cur != kNoNode) {
            rev.push_back(cur);
            cur = fwd_prev[cur];
        }
        std::reverse(rev.begin(), rev.end());
        for (NodeId n : rev) {
            expand_path(fwd_nodes.back(), n, fwd_nodes);
        }
    }

    // Backward path: meeting → t
    // In the backward search, bwd_prev[v] = u means we discovered v from u
    // by traversing the original downward edge v→u in reverse.  Following
    // bwd_prev FORWARD from meeting therefore traces the original path
    // meeting→...→t in the correct direction.
    {
        NodeId cur = meeting;
        while (true) {
            NodeId next = bwd_prev[cur];
            if (next == kNoNode) break;
            expand_path(cur, next, fwd_nodes);
            cur = next;
        }
    }

    // Convert node sequence to edge sequence
    Route r;
    r.is_valid = true;
    for (std::size_t i = 0; i + 1 < fwd_nodes.size(); ++i) {
        NodeId from = fwd_nodes[i], to = fwd_nodes[i + 1];
        bool found = false;
        for (EdgeId eid : graph_.out_edges(from)) {
            if (graph_.edges[eid].target == to) {
                r.edges.push_back(eid);
                r.estimated_dist_m  += graph_.edges[eid].length_m;
                r.estimated_time_s  += graph_.free_flow_time(eid);
                found = true;
                break;
            }
        }
        if (!found) { r.is_valid = false; break; }
    }
    return r;
}

// ── IRouter interface ─────────────────────────────────────────────────────────

Route CHRouter::route(const RoutingRequest& req) {
    if (cache_) {
        if (auto cached = cache_->get(req.origin, req.destination,
                                       req.mode, req.departure_time))
            return *cached;
    }
    Route r = ch_query(req.origin, req.destination, req.mode);
    if (r.is_valid && cache_)
        cache_->put(req.origin, req.destination, req.mode, req.departure_time, r);
    return r;
}

void CHRouter::batch_route(std::span<const RoutingRequest> reqs,
                             std::span<Route> out) {
    assert(reqs.size() == out.size());
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, reqs.size()),
        [&](const tbb::blocked_range<std::size_t>& rng) {
            for (std::size_t i = rng.begin(); i < rng.end(); ++i)
                out[i] = route(reqs[i]);
        });
}

void CHRouter::update_costs(std::span<const EdgeId> changed_edges) {
    if (cache_) cache_->invalidate_edges(changed_edges);
}

// ── Serialization ─────────────────────────────────────────────────────────────

static constexpr uint64_t kChMagic   = 0x4E4F4D4143480001ULL; // "NOMACH\x00\x01"
static constexpr uint32_t kChVersion = 2;

void CHRouter::save(const std::filesystem::path& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot open CH file for writing: " + path.string());

    out.write(reinterpret_cast<const char*>(&kChMagic),   sizeof(kChMagic));
    out.write(reinterpret_cast<const char*>(&kChVersion), sizeof(kChVersion));

    uint32_t N   = static_cast<uint32_t>(ch_.level.size());
    uint32_t nsc = static_cast<uint32_t>(ch_.shortcuts.size());
    uint32_t E   = static_cast<uint32_t>(graph_.num_edges()); // graph fingerprint
    out.write(reinterpret_cast<const char*>(&N),   sizeof(N));
    out.write(reinterpret_cast<const char*>(&nsc), sizeof(nsc));
    out.write(reinterpret_cast<const char*>(&E),   sizeof(E));
    out.write(reinterpret_cast<const char*>(ch_.level.data()),     N   * sizeof(uint32_t));
    out.write(reinterpret_cast<const char*>(ch_.shortcuts.data()), nsc * sizeof(Shortcut));
}

void CHRouter::load(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot open CH file: " + path.string());

    uint64_t magic = 0;
    in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    if (magic != kChMagic)
        throw std::runtime_error("CH cache is stale or corrupt (bad magic) — delete " +
                                  path.string() + " to rebuild");

    uint32_t ver = 0;
    in.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    if (ver != kChVersion)
        throw std::runtime_error("CH cache version mismatch (expected " +
                                  std::to_string(kChVersion) + ", got " +
                                  std::to_string(ver) + ") — delete " +
                                  path.string() + " to rebuild");

    uint32_t N = 0, nsc = 0, E_saved = 0;
    in.read(reinterpret_cast<char*>(&N),       sizeof(N));
    in.read(reinterpret_cast<char*>(&nsc),     sizeof(nsc));
    in.read(reinterpret_cast<char*>(&E_saved), sizeof(E_saved));

    if (N != graph_.num_nodes() || E_saved != graph_.num_edges())
        throw std::runtime_error("CH cache graph mismatch (cache: " +
                                  std::to_string(N) + " nodes / " +
                                  std::to_string(E_saved) + " edges, current graph: " +
                                  std::to_string(graph_.num_nodes()) + " / " +
                                  std::to_string(graph_.num_edges()) +
                                  ") — delete " + path.string() + " to rebuild");

    ch_.level.resize(N);
    ch_.shortcuts.resize(nsc);
    in.read(reinterpret_cast<char*>(ch_.level.data()),     N   * sizeof(uint32_t));
    in.read(reinterpret_cast<char*>(ch_.shortcuts.data()), nsc * sizeof(Shortcut));

    build_augmented_graph();
    preprocessed_ = true;
    spdlog::info("CHRouter: loaded {} shortcuts for {} nodes", nsc, N);
}

} // namespace nomad
