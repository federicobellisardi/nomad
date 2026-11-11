#include <nomad/routing/td_costs.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace nomad {

namespace {
constexpr char     kMagic[4] = {'N', 'T', 'D', 'C'};
constexpr uint32_t kVersion  = 1;
}

TdCostTable::TdCostTable(uint32_t num_edges, uint32_t num_bins, float bin_s, double t0)
    : E_(num_edges), B_(num_bins), bin_s_(bin_s), t0_(t0),
      tt_(static_cast<std::size_t>(num_edges) * num_bins,
          std::numeric_limits<float>::quiet_NaN()),
      n_(static_cast<std::size_t>(num_edges) * num_bins, 0u) {}

TdCostTable TdCostTable::load(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) throw std::runtime_error("TdCostTable: cannot open " + p.string());
    char magic[4];
    uint32_t ver = 0, E = 0, B = 0;
    float bin_s = 0.0f;
    double t0 = 0.0;
    in.read(magic, 4);
    in.read(reinterpret_cast<char*>(&ver), 4);
    if (!in || std::memcmp(magic, kMagic, 4) != 0 || ver != kVersion)
        throw std::runtime_error("TdCostTable: bad header in " + p.string());
    in.read(reinterpret_cast<char*>(&E), 4);
    in.read(reinterpret_cast<char*>(&B), 4);
    in.read(reinterpret_cast<char*>(&bin_s), 4);
    in.read(reinterpret_cast<char*>(&t0), 8);
    TdCostTable t(E, B, bin_s, t0);
    const std::size_t cells = static_cast<std::size_t>(E) * B;
    in.read(reinterpret_cast<char*>(t.tt_.data()), static_cast<std::streamsize>(cells * 4));
    in.read(reinterpret_cast<char*>(t.n_.data()),  static_cast<std::streamsize>(cells * 4));
    if (!in) throw std::runtime_error("TdCostTable: truncated file " + p.string());
    return t;
}

void TdCostTable::save(const std::filesystem::path& p) const {
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary);
    if (!out) throw std::runtime_error("TdCostTable: cannot write " + p.string());
    out.write(kMagic, 4);
    out.write(reinterpret_cast<const char*>(&kVersion), 4);
    out.write(reinterpret_cast<const char*>(&E_), 4);
    out.write(reinterpret_cast<const char*>(&B_), 4);
    out.write(reinterpret_cast<const char*>(&bin_s_), 4);
    out.write(reinterpret_cast<const char*>(&t0_), 8);
    out.write(reinterpret_cast<const char*>(tt_.data()), static_cast<std::streamsize>(tt_.size() * 4));
    out.write(reinterpret_cast<const char*>(n_.data()),  static_cast<std::streamsize>(n_.size() * 4));
}

TdCostRecorder::TdCostRecorder(uint32_t num_edges, double t0, double t1, float bin_s)
    : E_(num_edges),
      B_(static_cast<uint32_t>(std::max(1.0, std::ceil((t1 - t0) / bin_s)))),
      bin_s_(bin_s), t0_(t0),
      sum_(static_cast<std::size_t>(E_) * B_, 0.0),
      n_(static_cast<std::size_t>(E_) * B_, 0u) {}

void TdCostRecorder::record(EdgeId e, double enter_t, double leave_t) {
    if (e >= E_ || !(leave_t >= enter_t)) return;
    double k = (enter_t - t0_) / bin_s_;
    uint32_t b = k <= 0.0 ? 0u
               : (k >= static_cast<double>(B_ - 1) ? B_ - 1 : static_cast<uint32_t>(k));
    std::size_t i = static_cast<std::size_t>(e) * B_ + b;
    sum_[i] += leave_t - enter_t;
    ++n_[i];
}

TdCostTable TdCostRecorder::table() const {
    TdCostTable t(E_, B_, bin_s_, t0_);
    for (std::size_t i = 0; i < sum_.size(); ++i) {
        if (n_[i] == 0) continue;
        t.tt()[i] = static_cast<float>(sum_[i] / n_[i]);
        t.n()[i]  = n_[i];
    }
    return t;
}

} // namespace nomad
