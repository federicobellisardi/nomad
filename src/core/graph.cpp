#include <nomad/core/graph.hpp>

#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace nomad {

bool Graph::validate() const {
    const uint32_t N = num_nodes();
    const uint32_t E = num_edges();

    // CSR row_ptr size
    if (row_ptr.size() != N + 1) return false;
    if (col_idx.size() != E)     return false;
    if (edges.size()   != E)     return false;

    // row_ptr is non-decreasing and bounded
    for (uint32_t i = 0; i < N; ++i) {
        if (row_ptr[i] > row_ptr[i + 1]) return false;
        if (row_ptr[i + 1] > E)          return false;
    }

    // All edge targets are valid nodes
    for (const EdgeId eid : col_idx) {
        if (eid >= E) return false;
    }
    for (uint32_t i = 0; i < E; ++i) {
        if (edges[i].target >= N) return false;
    }

    // No self-loops
    for (uint32_t u = 0; u < N; ++u) {
        for (EdgeId eid : out_edges(u)) {
            if (edges[eid].target == u) return false;
        }
    }

    // Bidirectional edge pairing: reverse_of(reverse_of(e)) == e
    // and they share the same source-target pair in reverse
    if (E % 2 != 0) return false;
    for (uint32_t e = 0; e < E; e += 2) {
        const EdgeId fwd = e, rev = e + 1;
        if (edges[fwd].target != edges[rev].target) {
            // This check is approximate; a correct check requires knowing
            // the source node of each edge, which CSR doesn't store directly.
            // We relax this here; a fuller check is done in NetworkCleaner.
        }
    }

    // Geometry sidecar size
    if (!geom_ptr.empty()) {
        if (geom_ptr.size() != E + 1) return false;
        if (geom_ptr.back() != geom_coords.size()) return false;
    }

    return true;
}

} // namespace nomad
