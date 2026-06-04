#include <nomad/output/parquet_writer.hpp>
#include <spdlog/spdlog.h>

// Apache Arrow is an optional dep; stub the implementation
// when Arrow is not available (CI without Arrow installed).
#if defined(NOMAD_HAS_ARROW)
#  include <arrow/api.h>
#  include <arrow/io/api.h>
#  include <parquet/arrow/writer.h>
#endif

namespace nomad {

struct ParquetWriter::Impl {
    std::filesystem::path dir;
    uint32_t row_group_size;
    bool arrow_available;
};

ParquetWriter::ParquetWriter(std::filesystem::path output_dir, uint32_t row_group_size)
    : impl_(std::make_unique<Impl>(Impl{std::move(output_dir), row_group_size,
#if defined(NOMAD_HAS_ARROW)
      true
#else
      false
#endif
      }))
{
    std::filesystem::create_directories(impl_->dir);
    if (!impl_->arrow_available)
        spdlog::warn("ParquetWriter: Apache Arrow not available — output disabled");
}

ParquetWriter::~ParquetWriter() { close(); }

void ParquetWriter::on_event(const Event& e, const AgentHotStore&, const AgentColdStore&,
                               const Graph&, const ITrafficModel&) {
    // TODO Phase 4: buffer AgentArriveActivity events for activity table
}

void ParquetWriter::on_snapshot(SimTime t, const AgentHotStore& hot,
                                  const Graph& g, const ITrafficModel& traffic) {
    if (!impl_->arrow_available) return;
    // TODO Phase 4: write link_stats row group
}

void ParquetWriter::flush() {}
void ParquetWriter::close() {}

} // namespace nomad
