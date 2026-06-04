#include <catch2/catch_test_macros.hpp>

#include <nomad/core/agent.hpp>
#include <nomad/core/types.hpp>

using namespace nomad;

TEST_CASE("RouteStore: push and get round-trip", "[agent]") {
    RouteStore rs;
    std::vector<EdgeId> route = {0, 4, 8, 12, 16};
    rs.push_route(0, route);
    auto out = rs.get_route(0);
    REQUIRE(out == route);
}

TEST_CASE("RouteStore: empty route", "[agent]") {
    RouteStore rs;
    rs.push_route(0, {});
    auto out = rs.get_route(0);
    REQUIRE(out.empty());
}

TEST_CASE("RouteStore: multiple agents independent", "[agent]") {
    RouteStore rs;
    rs.push_route(0, {0, 2, 4});
    rs.push_route(1, {10, 20, 30, 40});
    rs.push_route(2, {100});

    REQUIRE(rs.get_route(0) == std::vector<EdgeId>({0, 2, 4}));
    REQUIRE(rs.get_route(1) == std::vector<EdgeId>({10, 20, 30, 40}));
    REQUIRE(rs.get_route(2) == std::vector<EdgeId>({100}));
}

TEST_CASE("RouteStore: large consecutive EdgeIds (small deltas)", "[agent]") {
    RouteStore rs;
    std::vector<EdgeId> route;
    for (int i = 0; i < 100; ++i) route.push_back(i * 2);
    rs.push_route(0, route);
    REQUIRE(rs.get_route(0) == route);
}

TEST_CASE("AgentHotStore: resize and check size", "[agent]") {
    AgentHotStore hot;
    hot.resize(1000);
    REQUIRE(hot.size() == 1000);
    REQUIRE(hot.current_edge.size() == 1000);
    REQUIRE(hot.state.size() == 1000);
}

TEST_CASE("AgentHotStore: push_back increments size", "[agent]") {
    AgentHotStore hot;
    hot.push_back(42, 100.0, 200.0, AgentMode::Car, AgentState::OnLink, 3);
    REQUIRE(hot.size() == 1);
    REQUIRE(hot.current_edge[0] == 42);
    REQUIRE(hot.mode[0] == AgentMode::Car);
    REQUIRE(hot.state[0] == AgentState::OnLink);
    REQUIRE(hot.route_pos[0] == 3);
}
