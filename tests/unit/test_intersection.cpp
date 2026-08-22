#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <nomad/network/intersection.hpp>

using namespace nomad;

TEST_CASE("effective_capacity: Simple intersection returns base capacity unchanged", "[intersection]") {
    REQUIRE(effective_capacity(1600.0f, IntersectionType::Simple) == 1600.0f);
}

TEST_CASE("effective_capacity: MotorwayJunction returns base capacity unchanged", "[intersection]") {
    REQUIRE(effective_capacity(2000.0f, IntersectionType::MotorwayJunction) == 2000.0f);
}

TEST_CASE("effective_capacity: AllWayStop returns base capacity unchanged (no derate defined)", "[intersection]") {
    // Only StopSign (not AllWayStop) is derated -- see effective_capacity()'s
    // switch. Documented as-is, not a gap this test needs to close.
    REQUIRE(effective_capacity(1600.0f, IntersectionType::AllWayStop) == 1600.0f);
}

TEST_CASE("effective_capacity: TrafficSignal with explicit phase derates by green/cycle * saturation", "[intersection]") {
    SignalPhase phase{};
    phase.cycle_s = 90.0f;
    phase.green_s = 40.0f;  // 44.4% green
    float expected = 1600.0f * (40.0f / 90.0f) * CapacityDerate::kSignalSaturation;
    REQUIRE(effective_capacity(1600.0f, IntersectionType::TrafficSignal, &phase) ==
            Catch::Approx(expected));
}

TEST_CASE("effective_capacity: TrafficSignal with no phase data falls back to 45% green default", "[intersection]") {
    float expected = 1600.0f * 0.45f * CapacityDerate::kSignalSaturation;
    REQUIRE(effective_capacity(1600.0f, IntersectionType::TrafficSignal, nullptr) ==
            Catch::Approx(expected));
}

TEST_CASE("effective_capacity: TrafficSignal with zero-length cycle falls back to 45% default "
          "(guards a real division-by-zero risk)", "[intersection]") {
    SignalPhase phase{};
    phase.cycle_s = 0.0f;
    phase.green_s = 0.0f;
    float expected = 1600.0f * 0.45f * CapacityDerate::kSignalSaturation;
    REQUIRE(effective_capacity(1600.0f, IntersectionType::TrafficSignal, &phase) ==
            Catch::Approx(expected));
}

TEST_CASE("effective_capacity: StopSign derates by kStopSignDerate", "[intersection]") {
    REQUIRE(effective_capacity(600.0f, IntersectionType::StopSign) ==
            Catch::Approx(600.0f * CapacityDerate::kStopSignDerate));
}

TEST_CASE("effective_capacity: Roundabout/MiniRoundabout derate by kRoundaboutDerate", "[intersection]") {
    REQUIRE(effective_capacity(1000.0f, IntersectionType::Roundabout) ==
            Catch::Approx(1000.0f * CapacityDerate::kRoundaboutDerate));
    REQUIRE(effective_capacity(1000.0f, IntersectionType::MiniRoundabout) ==
            Catch::Approx(1000.0f * CapacityDerate::kRoundaboutDerate));
}
