#pragma once

#include <nomad/core/types.hpp>
#include <nomad/routing/router.hpp>

#include <atomic>
#include <cstdint>
#include <list>
#include <optional>
#include <shared_mutex>
#include <span>
#include <unordered_map>

namespace nomad {

// ── Route cache ───────────────────────────────────────────────────────────────
// Two-level LRU cache indexed by (origin, destination, mode, TOD_bucket).
// The time-of-day bucket discretises departure_time into 15-minute slots
// (96 buckets/day), capturing temporal autocorrelation in urban demand without
// storing a full time-expanded graph.
//
// Concurrency: std::shared_mutex allows multiple concurrent readers (the
// common case during batch routing) and exclusive writes on insertion.
// Invalidation via update_costs() acquires an exclusive lock and removes
// all entries whose route contains any of the changed edges.
//
// Memory: each entry stores a Route (~160 bytes for 40-edge route), so
// 5M entries ≈ 800 MB. Default is 500k entries (~80 MB), tunable via ctor.
class RouteCache {
public:
    explicit RouteCache(std::size_t max_entries = 500'000);

    // Returns the cached route if present (marks it as recently used).
    std::optional<Route> get(NodeId origin, NodeId dest,
                              AgentMode mode, SimTime depart_time) const;

    // Insert a route. If the cache is full, evicts the LRU entry.
    void put(NodeId origin, NodeId dest,
              AgentMode mode, SimTime depart_time,
              Route route);

    // Remove all entries whose route passes through any of changed_edges.
    // Called after on_exit triggers a congestion state change.
    void invalidate_edges(std::span<const EdgeId> changed_edges);

    std::size_t size() const noexcept;
    void        clear() noexcept;

    // Cache statistics (for benchmarking / Python diagnostics)
    struct Stats {
        uint64_t hits;
        uint64_t misses;
        uint64_t evictions;
        uint64_t invalidations;
    };
    Stats stats() const noexcept;
    void  reset_stats() noexcept;

private:
    struct Key {
        NodeId    origin;
        NodeId    dest;
        uint8_t   mode;
        uint8_t   tod_bucket;  // departure_time / 900 % 96
        bool operator==(const Key&) const = default;
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept;
    };

    static uint8_t tod_bucket(SimTime t) noexcept {
        return static_cast<uint8_t>(static_cast<uint64_t>(t / 900.0) % 96);
    }

    using LruList = std::list<Key>;
    struct Entry {
        Route        route;
        LruList::iterator lru_it;
    };

    mutable std::shared_mutex                         mu_;
    std::unordered_map<Key, Entry, KeyHash>           map_;
    LruList                                           lru_;
    std::size_t                                       max_entries_;

    mutable std::atomic<uint64_t> hits_       {0};
    mutable std::atomic<uint64_t> misses_     {0};
    mutable std::atomic<uint64_t> evictions_  {0};
    mutable std::atomic<uint64_t> invalidations_{0};
};

} // namespace nomad
