#include <catch2/catch_test_macros.hpp>

#include <nomad/core/event.hpp>

using namespace nomad;

TEST_CASE("EventQueue: push and pop are FIFO by time", "[event]") {
    EventQueue q;
    q.push({10.0, 0, 0, EventType::AgentDepart, {}});
    q.push({5.0,  1, 0, EventType::AgentDepart, {}});
    q.push({7.0,  2, 0, EventType::AgentDepart, {}});

    REQUIRE(q.size() == 3);
    REQUIRE_FALSE(q.empty());

    Event e1 = q.pop();
    REQUIRE(e1.time == 5.0);
    Event e2 = q.pop();
    REQUIRE(e2.time == 7.0);
    Event e3 = q.pop();
    REQUIRE(e3.time == 10.0);
    REQUIRE(q.empty());
}

TEST_CASE("EventQueue: drain_until extracts correct events", "[event]") {
    EventQueue q;
    for (int i = 0; i < 10; ++i) {
        q.push({static_cast<SimTime>(i * 10), static_cast<AgentId>(i),
                0, EventType::AgentEnterLink, {}});
    }

    std::vector<Event> batch;
    q.drain_until(50.0, batch);  // should get events at t=0,10,20,30,40
    REQUIRE(batch.size() == 5);
    for (std::size_t i = 0; i < batch.size(); ++i) {
        REQUIRE(batch[i].time < 50.0);
    }
    // Remaining events: t=50,60,70,80,90
    REQUIRE(q.size() == 5);
}

TEST_CASE("EventQueue: drain_until output is sorted by time", "[event]") {
    EventQueue q;
    q.push({35.0, 0, 0, EventType::AgentDepart, {}});
    q.push({12.0, 1, 0, EventType::AgentDepart, {}});
    q.push({27.0, 2, 0, EventType::AgentDepart, {}});

    std::vector<Event> batch;
    q.drain_until(100.0, batch);
    REQUIRE(batch.size() == 3);
    for (std::size_t i = 1; i < batch.size(); ++i) {
        REQUIRE(batch[i-1].time <= batch[i].time);
    }
}

TEST_CASE("EventQueue: clear empties the queue", "[event]") {
    EventQueue q;
    q.push({1.0, 0, 0, EventType::AgentDepart, {}});
    q.push({2.0, 1, 0, EventType::AgentDepart, {}});
    q.clear();
    REQUIRE(q.empty());
    REQUIRE(q.size() == 0);
}

TEST_CASE("EventQueue: handles 1M events correctly", "[event]") {
    EventQueue q;
    constexpr int N = 1'000'000;
    for (int i = 0; i < N; ++i) {
        double t = static_cast<double>(i % 86400);
        q.push({t, static_cast<AgentId>(i % 100000), 0, EventType::AgentEnterLink, {}});
    }
    REQUIRE(q.size() == N);

    // Drain half
    std::vector<Event> batch;
    q.drain_until(43200.0, batch);
    // Should have drained events with t < 43200
    for (const auto& e : batch) REQUIRE(e.time < 43200.0);
}
