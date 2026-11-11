#pragma once
// Time-dependent link travel times for day-to-day (iterative) assignment.
//
// TdCostRecorder accumulates the travel time each car actually experienced
// on each link (exit time - entry time, so queueing and spillback waiting are
// included), binned by the entry time. Entries that never exit normally are
// still counted, as right-censored lower bounds: a teleport counts at the
// teleport time, and a car still on the link at the end of the run counts at
// that time.
//
// TdCostTable holds per-(edge, bin) mean travel times and is what the
// pre-router reads in the next iteration. Cells without data fall back to
// the free-flow time, and the cost is never below free flow.
//
// Binary format (little endian): "NTDC", u32 version=1, u32 E, u32 B,
// f32 bin_s, f64 t0, f32 tt[E*B] (NaN = no data), u32 n[E*B].
// Cell index = e * B + bin.

#include <nomad/core/types.hpp>

#include <cstdint>
#include <filesystem>
#include <vector>

namespace nomad {

class TdCostTable {
public:
    TdCostTable() = default;
    TdCostTable(uint32_t num_edges, uint32_t num_bins, float bin_s, double t0);

    // Travel time on e for a car entering at time t; falls back to ff when
    // the cell has no data, clamps below at ff and past the last bin.
    float cost(EdgeId e, double t, float ff) const noexcept {
        if (e >= E_ || B_ == 0) return ff;
        double k = (t - t0_) / bin_s_;
        uint32_t b = k <= 0.0 ? 0u
                   : (k >= static_cast<double>(B_ - 1) ? B_ - 1 : static_cast<uint32_t>(k));
        float v = tt_[static_cast<std::size_t>(e) * B_ + b];
        return (v == v && v > ff) ? v : ff;   // v == v rejects NaN
    }

    uint32_t num_edges() const noexcept { return E_; }
    uint32_t num_bins()  const noexcept { return B_; }
    float    bin_s()     const noexcept { return bin_s_; }
    double   t0()        const noexcept { return t0_; }

    std::vector<float>&          tt()       noexcept { return tt_; }
    const std::vector<float>&    tt() const noexcept { return tt_; }
    std::vector<uint32_t>&       n()        noexcept { return n_; }
    const std::vector<uint32_t>& n()  const noexcept { return n_; }

    static TdCostTable load(const std::filesystem::path& p);
    void save(const std::filesystem::path& p) const;

private:
    uint32_t E_{0}, B_{0};
    float    bin_s_{900.0f};
    double   t0_{0.0};
    std::vector<float>    tt_;
    std::vector<uint32_t> n_;
};

class TdCostRecorder {
public:
    TdCostRecorder(uint32_t num_edges, double t0, double t1, float bin_s);

    // Not thread-safe: called from the sequential event loop only.
    void record(EdgeId e, double enter_t, double leave_t);

    TdCostTable table() const;

private:
    uint32_t E_, B_;
    float    bin_s_;
    double   t0_;
    std::vector<double>   sum_;
    std::vector<uint32_t> n_;
};

} // namespace nomad
