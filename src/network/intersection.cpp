#include <nomad/network/intersection.hpp>

namespace nomad {

float effective_capacity(float base_capacity,
                           IntersectionType type,
                           const SignalPhase* phase) noexcept {
    switch (type) {
    case IntersectionType::TrafficSignal:
        if (phase && phase->cycle_s > 0.0f) {
            return base_capacity
                * (phase->green_s / phase->cycle_s)
                * CapacityDerate::kSignalSaturation;
        }
        // Default 45% green fraction if no phase data
        return base_capacity * 0.45f * CapacityDerate::kSignalSaturation;

    case IntersectionType::StopSign:
        return base_capacity * CapacityDerate::kStopSignDerate;

    case IntersectionType::Roundabout:
    case IntersectionType::MiniRoundabout:
        return base_capacity * CapacityDerate::kRoundaboutDerate;

    case IntersectionType::Simple:
    case IntersectionType::AllWayStop:
    case IntersectionType::MotorwayJunction:
    default:
        return base_capacity;
    }
}

} // namespace nomad
