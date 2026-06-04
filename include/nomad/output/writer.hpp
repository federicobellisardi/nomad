#pragma once

#include <nomad/core/agent.hpp>
#include <nomad/core/event.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/traffic/traffic_model.hpp>

#include <string_view>

namespace nomad {

// ── IOutputWriter ─────────────────────────────────────────────────────────────
// Called by the simulation engine at SnapshotDump events (every
// snapshot_interval_s seconds) and at simulation end.
//
// on_event() must be thread-safe: it may be called from the TBB thread pool.
// Implementations should buffer writes internally and flush asynchronously
// via a dedicated background thread to avoid blocking the simulation loop.
class IOutputWriter {
public:
    virtual ~IOutputWriter() = default;

    // Called for each event; type-specific handling is done internally.
    virtual void on_event(const Event&          event,
                           const AgentHotStore&  hot,
                           const AgentColdStore& cold,
                           const Graph&          graph,
                           const ITrafficModel&  traffic) = 0;

    // Called at the end of each sync window after flush_traffic_state().
    // Use for periodic link-state snapshots.
    virtual void on_snapshot(SimTime               t,
                              const AgentHotStore&  hot,
                              const Graph&          graph,
                              const ITrafficModel&  traffic) = 0;

    // Flush all buffers to disk. Blocking.
    virtual void flush() = 0;

    // Close output files cleanly.
    virtual void close() = 0;

    virtual std::string_view writer_name() const = 0;
};

} // namespace nomad
