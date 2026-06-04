#pragma once

#include <nomad/output/writer.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <thread>

namespace nomad {

// ── Parquet writer ────────────────────────────────────────────────────────────
// Writes three Apache Arrow Parquet tables to output_dir:
//
//  agent_trajectories.parquet
//    agent_id uint32, time float64, edge_id uint32, lon float32, lat float32,
//    mode uint8, state uint8
//    Written at every SnapshotDump event for all OnLink agents.
//    With store_traces=true, written at every AgentEnterLink/ExitLink event.
//
//  link_stats.parquet
//    time_bin float64, edge_id uint32, count uint32, travel_time_s float32,
//    inflow float32, outflow float32
//    Written at every SnapshotDump event for all active edges.
//
//  agent_activities.parquet
//    agent_id uint32, activity_type uint8, start_time float64,
//    end_time float64, node_id uint32
//    Written at AgentArriveActivity events.
//
// All writes are asynchronous: the simulation thread enqueues rows into a
// lock-free SPSC buffer; a background thread drains and writes row groups
// of row_group_size rows at a time.
class ParquetWriter final : public IOutputWriter {
public:
    explicit ParquetWriter(std::filesystem::path output_dir,
                            uint32_t row_group_size = 500'000);
    ~ParquetWriter() override;

    void on_event  (const Event&, const AgentHotStore&, const AgentColdStore&,
                    const Graph&, const ITrafficModel&) override;
    void on_snapshot(SimTime t, const AgentHotStore&,
                     const Graph&, const ITrafficModel&) override;
    void flush () override;
    void close () override;

    std::string_view writer_name() const override { return "parquet"; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace nomad
