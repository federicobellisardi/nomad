#include <nomad/output/geojson_writer.hpp>
#include <fstream>
#include <iomanip>
#include <spdlog/spdlog.h>

namespace nomad {

struct GeoJsonWriter::Impl {
    std::filesystem::path dir;
    float interval;
};

GeoJsonWriter::GeoJsonWriter(std::filesystem::path output_dir, float snapshot_interval_s)
    : impl_(std::make_unique<Impl>(Impl{std::move(output_dir), snapshot_interval_s})),
      snapshot_interval_s_(snapshot_interval_s) {
    std::filesystem::create_directories(impl_->dir);
}

GeoJsonWriter::~GeoJsonWriter() { close(); }

void GeoJsonWriter::on_event(const Event&, const AgentHotStore&,
                               const AgentColdStore&, const Graph&, const ITrafficModel&) {}

void GeoJsonWriter::on_snapshot(SimTime t, const AgentHotStore&,
                                  const Graph& g, const ITrafficModel& traffic) {
    if (last_snapshot_ >= 0.0 && t - last_snapshot_ < snapshot_interval_s_) return;
    last_snapshot_ = t;

    char buf[64];
    std::snprintf(buf, sizeof(buf), "network_state_%06d.geojson", static_cast<int>(t));
    auto path = impl_->dir / buf;
    std::ofstream f(path);
    if (!f) { spdlog::warn("GeoJsonWriter: cannot open {}", path.string()); return; }

    f << std::fixed << std::setprecision(6);
    f << R"({"type":"FeatureCollection","features":[)";

    const auto states = traffic.link_states();
    bool first = true;
    for (uint32_t e = 0; e < g.num_edges(); ++e) {
        float occ = states[e].occupancy.load();
        if (occ == 0.0f) continue;

        const EdgeData& ed = g.edges[e];
        NodeId src = kInvalidNode, tgt = ed.target;
        // Find source node (CSR: scan row_ptr)
        for (uint32_t u = 0; u + 1 < g.row_ptr.size(); ++u) {
            for (uint32_t idx = g.row_ptr[u]; idx < g.row_ptr[u+1]; ++idx) {
                if (g.col_idx[idx] == e) { src = u; break; }
            }
            if (src != kInvalidNode) break;
        }
        if (src == kInvalidNode || tgt >= g.num_nodes()) continue;

        float ff = g.free_flow_time(e);
        float tt = traffic.current_travel_time(e);
        float cong = (ff > 0) ? std::min((tt / ff) - 1.0f, 1.0f) : 0.0f;

        if (!first) f << ',';
        first = false;

        f << R"({"type":"Feature","geometry":{"type":"LineString","coordinates":[)";
        f << '[' << g.nodes[src].lon << ',' << g.nodes[src].lat << ']';
        f << ',' << '[' << g.nodes[tgt].lon << ',' << g.nodes[tgt].lat << ']';
        f << R"(]},"properties":{)";
        f << R"("edge_id":)" << e;
        f << R"(,"road_class":)" << static_cast<int>(ed.road_class);
        f << R"(,"congestion":)" << cong;
        f << R"(,"travel_time_s":)" << tt;
        f << R"(,"count":)" << static_cast<int>(occ);
        f << R"(,"free_flow_time_s":)" << ff;
        if (!g.way_names.empty() && ed.way_meta_idx < g.way_names.size())
            f << R"(,"name":")" << g.way_names[ed.way_meta_idx] << '"';
        f << "}}";
    }
    f << "]}";
    spdlog::debug("GeoJsonWriter: wrote {}", path.string());
}

void GeoJsonWriter::write_static_network(const Graph& g, const std::filesystem::path& path) {
    std::ofstream f(path);
    if (!f) { spdlog::error("GeoJsonWriter: cannot write static network to {}", path.string()); return; }
    f << std::fixed << std::setprecision(6);
    f << R"({"type":"FeatureCollection","features":[)";
    bool first = true;
    for (uint32_t u = 0; u < g.num_nodes(); ++u) {
        for (EdgeId eid : g.out_edges(u)) {
            const EdgeData& ed = g.edges[eid];
            NodeId v = ed.target;
            if (v >= g.num_nodes()) continue;
            if (!first) f << ',';
            first = false;
            f << R"({"type":"Feature","geometry":{"type":"LineString","coordinates":[[)";
            f << g.nodes[u].lon << ',' << g.nodes[u].lat << "],[";
            f << g.nodes[v].lon << ',' << g.nodes[v].lat << R"(]]},"properties":{"edge_id":)" << eid;
            f << R"(,"road_class":)" << static_cast<int>(ed.road_class);
            f << R"(,"length_m":)" << ed.length_m;
            f << R"(,"speed_ms":)" << ed.free_flow_speed;
            f << "}}";
        }
    }
    f << "]}";
    spdlog::info("GeoJsonWriter: wrote static network to {}", path.string());
}

void GeoJsonWriter::flush() {}
void GeoJsonWriter::close() {}

} // namespace nomad
