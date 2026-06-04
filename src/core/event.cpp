#include <nomad/core/event.hpp>

#include <algorithm>
#include <stdexcept>

namespace nomad {

EventQueue::EventQueue(SimTime bucket_width_s)
    : bucket_width_(bucket_width_s)
{}

void EventQueue::push(Event e) {
    int b = time_to_bucket(e.time);
    if (b < 0 || b >= kTotalBuckets) {
        overflow_.push(e);
    } else {
        calendar_[b].push_back(e);
    }
    ++size_;
}

Event EventQueue::pop() {
    // Drain overflow entries that now fall within the calendar
    while (!overflow_.empty()) {
        const Event& top = overflow_.top();
        int b = time_to_bucket(top.time);
        if (b >= 0 && b < kTotalBuckets) {
            calendar_[b].push_back(top);
            overflow_.pop();
        } else {
            break;
        }
    }

    // Advance current_bucket_ until we find a non-empty bucket
    while (current_bucket_ < kTotalBuckets && calendar_[current_bucket_].empty()) {
        ++current_bucket_;
    }

    if (current_bucket_ >= kTotalBuckets) {
        if (overflow_.empty()) {
            throw std::runtime_error("EventQueue::pop() called on empty queue");
        }
        Event e = overflow_.top();
        overflow_.pop();
        --size_;
        return e;
    }

    // Find minimum event in current bucket (insertion-unsorted)
    auto& bucket = calendar_[current_bucket_];
    auto min_it  = std::min_element(bucket.begin(), bucket.end(),
                                     [](const Event& a, const Event& b){
                                         return a.time < b.time;
                                     });
    Event e = *min_it;
    // Swap-and-pop for O(1) removal
    *min_it = bucket.back();
    bucket.pop_back();
    --size_;
    return e;
}

bool EventQueue::empty() const noexcept {
    return size_ == 0;
}

void EventQueue::drain_until(SimTime until_time, std::vector<Event>& out) {
    int max_bucket = time_to_bucket(until_time);

    // Move eligible overflow events into calendar
    while (!overflow_.empty()) {
        const Event& top = overflow_.top();
        if (top.time >= until_time) break;
        int b = time_to_bucket(top.time);
        calendar_[b < 0 ? 0 : (b >= kTotalBuckets ? kTotalBuckets-1 : b)]
            .push_back(top);
        const_cast<std::priority_queue<Event, std::vector<Event>,
            std::greater<Event>>&>(overflow_).pop();
    }

    // Collect all events from current_bucket_ to max_bucket.
    // For buckets strictly before max_bucket: drain entirely (all events are < until_time).
    // For max_bucket: only drain events with time < until_time; leave the rest.
    for (int b = current_bucket_; b < max_bucket && b < kTotalBuckets; ++b) {
        out.insert(out.end(), calendar_[b].begin(), calendar_[b].end());
        size_ -= calendar_[b].size();
        calendar_[b].clear();
    }
    // Boundary bucket: filter by time
    if (max_bucket < kTotalBuckets) {
        auto& bkt = calendar_[max_bucket];
        std::vector<Event> keep;
        for (auto& e : bkt) {
            if (e.time < until_time) {
                out.push_back(e);
                --size_;
            } else {
                keep.push_back(e);
            }
        }
        bkt = std::move(keep);
    }

    // Sort the batch by time for deterministic processing order
    std::sort(out.begin(), out.end(), [](const Event& a, const Event& b){
        return a.time < b.time;
    });

    current_bucket_ = max_bucket;
}

void EventQueue::clear() {
    for (auto& bucket : calendar_) bucket.clear();
    while (!overflow_.empty()) overflow_.pop();
    size_ = 0;
    current_bucket_ = 0;
}

} // namespace nomad
