#include <nomad/core/agent.hpp>

#include <cassert>
#include <stdexcept>

namespace nomad {

void RouteStore::push_route(AgentId agent, const std::vector<EdgeId>& route) {
    // Resize if needed
    if (agent >= offsets.size()) {
        offsets    .resize(agent + 1, 0);
        base_edges .resize(agent + 1, kInvalidEdge);
    }

    if (route.empty()) {
        offsets[agent]    = static_cast<uint32_t>(flat_data.size());
        base_edges[agent] = kInvalidEdge;
        flat_data.push_back(0); // length = 0
        return;
    }

    offsets[agent]    = static_cast<uint32_t>(flat_data.size());
    base_edges[agent] = route[0];

    // Store length
    flat_data.push_back(static_cast<uint16_t>(route.size()));

    // Delta-encode: first delta is always 0 (base edge)
    flat_data.push_back(0);
    for (std::size_t i = 1; i < route.size(); ++i) {
        int64_t delta = static_cast<int64_t>(route[i]) -
                         static_cast<int64_t>(route[i-1]);
        // Clamp delta to uint16_t range; paths violating this are stored as
        // absolute values in a fall-through path (rare for normal routes)
        if (delta < 0 || delta > 65535) {
            // Fall back to absolute encoding with a sentinel
            // (sentinel = 0xFFFF, next two uint16_t form a uint32_t absolute)
            flat_data.push_back(0xFFFF);
            flat_data.push_back(static_cast<uint16_t>(route[i] & 0xFFFF));
            flat_data.push_back(static_cast<uint16_t>(route[i] >> 16));
        } else {
            flat_data.push_back(static_cast<uint16_t>(delta));
        }
    }
}

std::vector<EdgeId> RouteStore::get_route(AgentId agent) const {
    if (agent >= offsets.size() || base_edges[agent] == kInvalidEdge) {
        return {};
    }

    uint32_t off     = offsets[agent];
    uint16_t length  = flat_data[off];
    if (length == 0) return {};

    std::vector<EdgeId> route;
    route.reserve(length);
    route.push_back(base_edges[agent]);

    EdgeId prev = base_edges[agent];
    std::size_t i = off + 2; // skip length + first delta (0)
    while (route.size() < length) {
        uint16_t delta = flat_data[i++];
        if (delta == 0xFFFF) {
            // Absolute encoding
            uint16_t lo = flat_data[i++];
            uint16_t hi = flat_data[i++];
            EdgeId abs  = (static_cast<EdgeId>(hi) << 16) | lo;
            route.push_back(abs);
            prev = abs;
        } else {
            prev += delta;
            route.push_back(prev);
        }
    }
    return route;
}

void RouteStore::replace_suffix(AgentId agent, uint16_t from_pos,
                                  const std::vector<EdgeId>& new_suffix) {
    // Get current route and replace suffix from from_pos onward
    std::vector<EdgeId> current = get_route(agent);
    current.resize(from_pos);
    current.insert(current.end(), new_suffix.begin(), new_suffix.end());
    push_route(agent, current);
}

} // namespace nomad
