#pragma once

#include <nomad/core/types.hpp>

#include <array>
#include <cstdint>
#include <queue>
#include <vector>

namespace nomad {

// ── Event ─────────────────────────────────────────────────────────────────────
// 24 bytes (double forces 8-byte alignment; 3 bytes trailing padding).
struct Event {
    SimTime   time;     // 8  absolute simulation time [s since midnight]
    AgentId   agent;    // 4  kNoAgent for system events
    uint32_t  payload;  // 4  EdgeId, NodeId, or type-specific index
    EventType type;     // 1
    uint8_t   _pad[7];  // 7  alignment padding to multiple of 8

    bool operator>(const Event& o) const noexcept { return time > o.time; }
};
static_assert(sizeof(Event) == 24);

// ── Two-level calendar queue ──────────────────────────────────────────────────
// Urban simulation has highly clustered event times (departure waves at 7-9am
// and 5-7pm). A calendar queue with 1-second fine buckets outperforms a binary
// heap for clustered distributions by ~3-5× at 200k events/second.
//
// Structure:
//   - kCoarseBuckets × kFineBuckets = 1440 × 60 = 86400 buckets (one per second)
//   - Each bucket is an unsorted vector of events (sorted at drain time)
//   - Events > 24h in the future go to the binary heap overflow
//
// Thread-safety: this class is NOT thread-safe. The simulation engine is
// responsible for draining events into a batch before parallel dispatch.
class EventQueue {
public:
    explicit EventQueue(SimTime bucket_width_s = 1.0);

    void   push(Event e);
    Event  pop();           // returns event with smallest time
    bool   empty() const noexcept;
    std::size_t size() const noexcept { return size_; }

    // Drain all events in [0, until_time) into out, sorted by time.
    // Efficient: only scans buckets in the relevant time range.
    void drain_until(SimTime until_time, std::vector<Event>& out);

    void clear();

private:
    static constexpr int kTotalBuckets = 86400; // one per second of 24h

    int time_to_bucket(SimTime t) const noexcept {
        int b = static_cast<int>(t / bucket_width_);
        return (b < 0) ? 0 : (b >= kTotalBuckets ? kTotalBuckets - 1 : b);
    }

    std::array<std::vector<Event>, kTotalBuckets> calendar_;
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> overflow_;
    SimTime     bucket_width_;
    std::size_t size_{0};
    int         current_bucket_{0};
};

} // namespace nomad
