#include <nomad/output/geojson_writer.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <spdlog/spdlog.h>

namespace nomad {

struct GeoJsonWriter::Impl {
    std::filesystem::path dir;
    float interval;
    // edge_src[e] = source node of edge e (built once on first snapshot)
    std::vector<NodeId> edge_src;
};

GeoJsonWriter::GeoJsonWriter(std::filesystem::path output_dir, float snapshot_interval_s)
    : impl_(std::make_unique<Impl>(Impl{std::move(output_dir), snapshot_interval_s, {}})),
      snapshot_interval_s_(snapshot_interval_s) {
    std::filesystem::create_directories(impl_->dir);
}

GeoJsonWriter::~GeoJsonWriter() { close(); }

void GeoJsonWriter::on_event(const Event&, const AgentHotStore&,
                               const AgentColdStore&, const Graph&, const ITrafficModel&) {}

void GeoJsonWriter::on_snapshot(SimTime t, const AgentHotStore&,
                                  const Graph& g, const ITrafficModel& traffic) {
    // Build edge→source mapping once
    if (impl_->edge_src.empty()) {
        impl_->edge_src.assign(g.num_edges(), kInvalidNode);
        for (uint32_t u = 0; u < g.num_nodes(); ++u)
            for (EdgeId eid : g.out_edges(u))
                impl_->edge_src[eid] = u;
        write_static_network(g, impl_->dir / "static_network.geojson");
    }

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
        NodeId src = impl_->edge_src[e];
        NodeId tgt = ed.target;
        if (src == kInvalidNode || tgt >= g.num_nodes()) continue;

        float ff = g.free_flow_time(e);
        float tt = traffic.current_travel_time(e);
        // Congestion ∈ [0, 1]: normalise BPR delay by its max at vc=2 cap (0.15×2⁴=2.4).
        // congestion = 0  → free-flow;  congestion = 1 → max BPR penalty (3.4× free-flow).
        constexpr float kBprMaxDelay = 2.4f;
        float cong = (ff > 0 && kBprMaxDelay > 0)
            ? std::min((tt / ff - 1.0f) / kBprMaxDelay, 1.0f)
            : 0.0f;
        if (cong < 0.0f) cong = 0.0f;

        if (!first) f << ',';
        first = false;

        f << R"({"type":"Feature","geometry":{"type":"LineString","coordinates":[)";
        f << '[' << g.nodes[src].lon << ',' << g.nodes[src].lat << ']';
        // Intermediate shape points (OSM geometry nodes between src and tgt)
        if (!g.geom_ptr.empty() && e + 1 < g.geom_ptr.size()) {
            for (uint32_t gi = g.geom_ptr[e]; gi < g.geom_ptr[e + 1]; ++gi)
                f << ",[" << g.geom_coords[gi].first << ',' << g.geom_coords[gi].second << ']';
        }
        f << ",[" << g.nodes[tgt].lon << ',' << g.nodes[tgt].lat << ']';
        f << R"(]},"properties":{)";
        f << R"("edge_id":)"         << e;
        f << R"(,"road_class":)"     << static_cast<int>(ed.road_class);
        f << R"(,"count":)"          << static_cast<int>(occ);
        f << R"(,"congestion":)"     << cong;
        f << R"(,"travel_time_s":)"  << tt;
        f << R"(,"free_flow_time_s":)" << ff;
        f << R"(,"length_m":)"       << ed.length_m;
        // Speed [m/s]: distance / current travel time
        float speed = (tt > 0.0f) ? ed.length_m / tt : ed.free_flow_speed;
        f << R"(,"speed_ms":)"       << speed;
        f << R"(,"speed_kmh":)"      << speed * 3.6f;
        f << R"(,"capacity_veh_h":)" << ed.capacity;
        // Flow via Little's Law capped at link capacity (short links with tt≈0 would explode)
        float flow = (tt > 0.0f) ? std::min(occ * 3600.0f / tt, ed.capacity) : 0.0f;
        f << R"(,"flow_veh_h":)"     << flow;
        // Density [veh/km]
        float density = (ed.length_m > 0.0f) ? occ / (ed.length_m * 0.001f) : 0.0f;
        f << R"(,"density_veh_km":)" << density;
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
