#pragma once

#include <nomad/core/types.hpp>
#include <nomad/network/multilayer_graph.hpp>

#include <functional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace nomad {

// ── Generalized cost ─────────────────────────────────────────────────────────
// The generalized cost is the composite measure used for mode choice.
// Each component is converted to seconds equivalent via value-of-time (VOT).
struct GeneralizedCost {
    float travel_time_s   = 0.0f;
    float wait_time_s     = 0.0f;   // for PT: waiting at stop
    float walk_time_s     = 0.0f;   // access/egress walking
    float transfer_pen_s  = 0.0f;   // penalty per transfer
    float monetary_cost   = 0.0f;   // €, converted to s via VOT
    float comfort_factor  = 1.0f;   // mode-specific discomfort multiplier

    float total(float vot_eur_per_hour = 12.0f) const noexcept {
        return travel_time_s * comfort_factor
             + wait_time_s * 1.5f            // waiting is 1.5× more aversive
             + walk_time_s * 2.0f            // walking is 2× more aversive
             + transfer_pen_s
             + monetary_cost * 3600.0f / vot_eur_per_hour;
    }
};

// ── Mode alternative ──────────────────────────────────────────────────────────
struct ModeAlternative {
    AgentMode       mode;
    GeneralizedCost cost;
    bool            available;  // mode is physically accessible for this OD
};

// ── Mode choice model interface ───────────────────────────────────────────────
// All mode choice models implement this interface.
//
// Design comparison:
//   ActivitySim: MNL (Multinomial Logit) with ASC (Alternative-Specific Constants)
//     and extensive socioeconomic variables. Gold standard for activity-based.
//   MATSim: TeleportationEngine (utility-based mode choice from plans).
//   SUMO: TripRouter (shortest path per mode, no utility maximisation).
//   A/B Street: mode specified per-trip, no dynamic choice.
//
// nomad default: MNL with configurable beta coefficients and ASCs.
// This is the minimum requirement for publication-quality mode choice.
// ActivitySim output can be used as a pre-computed choice (ModeFromPlan).
class IModeChoiceModel {
public:
    virtual ~IModeChoiceModel() = default;

    // Given a list of mode alternatives, return the chosen mode.
    virtual AgentMode choose(const std::vector<ModeAlternative>& alts,
                              std::mt19937& rng) const = 0;

    virtual std::string_view model_name() const = 0;
};

// ── Multinomial Logit (MNL) ───────────────────────────────────────────────────
// P(mode | alts) ∝ exp(V(mode)) where V(mode) is the utility function.
// V(mode) = ASC_mode + β_time × time + β_cost × cost + ...
//
// This is the most widely used mode choice model in transportation research.
// Parameters are typically estimated from stated/revealed preference surveys.
class MnlModeChoice final : public IModeChoiceModel {
public:
    struct Params {
        // Alternative-specific constants (reference mode = Car = 0)
        float asc_car     = 0.0f;
        float asc_bike    = -0.5f;  // bike is less preferred ceteris paribus
        float asc_walk    = -0.8f;
        float asc_transit = -0.3f;

        // Beta coefficients (negative: higher cost = lower utility)
        float beta_time   = -0.015f;  // per second
        float beta_cost   = -0.20f;   // per euro
        float beta_wait   = -0.025f;  // per second (waiting)
        float beta_walk   = -0.030f;  // per second (walking)
        float beta_transfer= -300.0f; // per transfer

        float vot_eur_per_hour = 12.0f;
    };

    explicit MnlModeChoice();
    explicit MnlModeChoice(Params p);

    AgentMode choose(const std::vector<ModeAlternative>& alts,
                      std::mt19937& rng) const override;

    std::string_view model_name() const override { return "MNL"; }

private:
    float utility(const ModeAlternative& alt) const noexcept;
    Params p_;
};

// ── Mode from plan (ActivitySim connector) ────────────────────────────────────
// Mode is pre-determined from an external demand model (e.g., ActivitySim).
// Returns the mode stored in the agent's ActivityPlan.
class ModeFromPlan final : public IModeChoiceModel {
public:
    AgentMode choose(const std::vector<ModeAlternative>& alts,
                      std::mt19937&) const override {
        for (const auto& a : alts) if (a.available) return a.mode;
        return AgentMode::Walk;
    }
    std::string_view model_name() const override { return "from_plan"; }
};

} // namespace nomad
