#pragma once

#include <nomad/core/types.hpp>

#include <cmath>
#include <random>
#include <vector>

namespace nomad {

// ── Departure time sampler ────────────────────────────────────────────────────
// Reusable across demand models. Supports three distributions:
//
//  LogNormal  — right-skewed, appropriate for peak-hour commute waves
//  Empirical  — arbitrary CDF loaded from data (e.g., NHTS household survey)
//  Uniform    — simple baseline / testing
//
// All distributions are truncated to [0, 86400) seconds (one day).
class DepartureSampler {
public:
    enum class Distribution { LogNormal, Empirical, Uniform };

    // LogNormal: mean and std are in seconds (internally converted to μ, σ of log)
    static DepartureSampler lognormal(float mean_s, float std_s) {
        DepartureSampler s;
        s.dist_ = Distribution::LogNormal;
        s.mean_ = mean_s;
        s.std_  = std_s;
        return s;
    }

    // Empirical: provide (time_s, cumulative_probability) pairs, sorted by time
    static DepartureSampler empirical(std::vector<std::pair<float,float>> cdf) {
        DepartureSampler s;
        s.dist_ = Distribution::Empirical;
        s.cdf_  = std::move(cdf);
        return s;
    }

    // Uniform in [a_s, b_s)
    static DepartureSampler uniform(float a_s, float b_s) {
        DepartureSampler s;
        s.dist_ = Distribution::Uniform;
        s.mean_ = a_s;
        s.std_  = b_s;
        return s;
    }

    SimTime sample(std::mt19937& rng) const;

private:
    DepartureSampler() = default;

    Distribution dist_{Distribution::Uniform};
    float mean_{0.0f}, std_{86400.0f};
    std::vector<std::pair<float,float>> cdf_;
};

inline SimTime DepartureSampler::sample(std::mt19937& rng) const {
    constexpr float kDayS = 86400.0f;
    SimTime t = 0.0;
    switch (dist_) {
    case Distribution::Uniform: {
        std::uniform_real_distribution<float> d(mean_, std_);
        t = d(rng);
        break;
    }
    case Distribution::LogNormal: {
        // Convert normal mean/std to log-space parameters
        float var    = std_ * std_;
        float mu_log = std::log(mean_ * mean_ / std::sqrt(var + mean_ * mean_));
        float si_log = std::sqrt(std::log(1.0f + var / (mean_ * mean_)));
        std::lognormal_distribution<float> d(mu_log, si_log);
        t = d(rng);
        break;
    }
    case Distribution::Empirical: {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        float p = u(rng);
        // Linear interpolation in CDF
        for (std::size_t i = 1; i < cdf_.size(); ++i) {
            if (p <= cdf_[i].second) {
                float t0 = cdf_[i-1].first, t1 = cdf_[i].first;
                float p0 = cdf_[i-1].second, p1 = cdf_[i].second;
                t = t0 + (t1 - t0) * (p - p0) / (p1 - p0);
                break;
            }
        }
        break;
    }
    }
    // Truncate to [0, 86400)
    if (t < 0.0f) t = 0.0f;
    if (t >= kDayS) t = kDayS - 1.0f;
    return static_cast<SimTime>(t);
}

} // namespace nomad
