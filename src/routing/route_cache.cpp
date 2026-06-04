#include <nomad/routing/route_cache.hpp>

#include <algorithm>
#include <functional>
#include <mutex>

namespace nomad {

std::size_t RouteCache::KeyHash::operator()(const Key& k) const noexcept {
    // FNV-1a-inspired hash combining the four fields
    std::size_t h = 2166136261ULL;
    auto mix = [&](std::size_t v) {
        h ^= v;
        h *= 16777619ULL;
    };
    mix(k.origin);
    mix(k.dest);
    mix(k.mode);
    mix(k.tod_bucket);
    return h;
}

RouteCache::RouteCache(std::size_t max_entries) : max_entries_(max_entries) {}

std::optional<Route> RouteCache::get(NodeId origin, NodeId dest,
                                       AgentMode mode, SimTime t) const {
    Key k{origin, dest, static_cast<uint8_t>(mode), tod_bucket(t)};
    std::shared_lock lock(mu_);
    auto it = map_.find(k);
    if (it == map_.end()) {
        ++misses_;
        return std::nullopt;
    }
    ++hits_;
    // Move to front of LRU (requires write lock — downgrade not supported in C++17)
    // For simplicity in this phase, skip LRU promotion on reads.
    Route r = it->second.route;
    r.is_cached = true;
    return r;
}

void RouteCache::put(NodeId origin, NodeId dest, AgentMode mode,
                      SimTime t, Route route) {
    Key k{origin, dest, static_cast<uint8_t>(mode), tod_bucket(t)};
    std::unique_lock lock(mu_);

    auto it = map_.find(k);
    if (it != map_.end()) {
        // Update existing: move to front
        lru_.erase(it->second.lru_it);
        lru_.push_front(k);
        it->second = {std::move(route), lru_.begin()};
        return;
    }

    // Evict LRU if at capacity
    if (map_.size() >= max_entries_) {
        Key evict_key = lru_.back();
        lru_.pop_back();
        map_.erase(evict_key);
        ++evictions_;
    }

    lru_.push_front(k);
    map_.emplace(k, Entry{std::move(route), lru_.begin()});
}

void RouteCache::invalidate_edges(std::span<const EdgeId> changed_edges) {
    if (changed_edges.empty()) return;
    std::unique_lock lock(mu_);

    std::vector<Key> to_remove;
    for (const auto& [k, entry] : map_) {
        const Route& r = entry.route;
        for (EdgeId e : changed_edges) {
            if (std::find(r.edges.begin(), r.edges.end(), e) != r.edges.end()) {
                to_remove.push_back(k);
                break;
            }
        }
    }
    for (const Key& k : to_remove) {
        lru_.erase(map_.at(k).lru_it);
        map_.erase(k);
        ++invalidations_;
    }
}

std::size_t RouteCache::size() const noexcept {
    std::shared_lock lock(mu_);
    return map_.size();
}

void RouteCache::clear() noexcept {
    std::unique_lock lock(mu_);
    map_.clear();
    lru_.clear();
}

RouteCache::Stats RouteCache::stats() const noexcept {
    return {hits_.load(), misses_.load(), evictions_.load(), invalidations_.load()};
}

void RouteCache::reset_stats() noexcept {
    hits_ = misses_ = evictions_ = invalidations_ = 0;
}

} // namespace nomad
