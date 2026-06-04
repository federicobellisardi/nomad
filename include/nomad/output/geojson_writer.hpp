#pragma once

#include <nomad/output/writer.hpp>

#include <filesystem>
#include <memory>

namespace nomad {

// ── GeoJSON writer ────────────────────────────────────────────────────────────
// Writes link-state snapshots as GeoJSON FeatureCollections for
// deck.gl / kepler.gl / Leaflet visualization.
//
// Output file: {output_dir}/network_state_{timestamp}.geojson
// Produced every snapshot_interval_s seconds (default 300s).
//
// GeoJSON schema per feature:
//   type: "Feature"
//   geometry: { type: "LineString", coordinates: [[lon,lat], ...] }
//   properties: {
//     edge_id: uint32,
//     road_class: uint8,
//     name: string,
//     congestion: float,       -- 0=free-flow, 1=jammed
//     travel_time_s: float,
//     count: uint32,
//     free_flow_time_s: float
//   }
//
// Only edges with count > 0 are written (sparse output for large networks).
// For static network export (no simulation), use write_static_network().
class GeoJsonWriter final : public IOutputWriter {
public:
    explicit GeoJsonWriter(std::filesystem::path output_dir,
                            float snapshot_interval_s = 300.0f);
    ~GeoJsonWriter() override;

    void on_event  (const Event&, const AgentHotStore&, const AgentColdStore&,
                    const Graph&, const ITrafficModel&) override;
    void on_snapshot(SimTime t, const AgentHotStore&,
                     const Graph&, const ITrafficModel&) override;
    void flush() override;
    void close() override;

    std::string_view writer_name() const override { return "geojson"; }

    // Export the full static network as a single GeoJSON file (no simulation
    // required). Useful for network inspection in QGIS or kepler.gl.
    void write_static_network(const Graph& g,
                               const std::filesystem::path& path);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    float snapshot_interval_s_;
    SimTime last_snapshot_{-1.0};
};

} // namespace nomad
