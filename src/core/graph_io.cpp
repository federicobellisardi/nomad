#include <nomad/core/graph.hpp>

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace nomad {

// Binary format: "NOMG" magic + uint32 version + packed vectors.
// All integers are little-endian (host byte order — nomad is single-machine).
static constexpr char     kMagic[4] = {'N','O','M','G'};
static constexpr uint32_t kVersion  = 1;

namespace {

template<typename T>
void write_vec(std::ostream& out, const std::vector<T>& v) {
    uint32_t n = static_cast<uint32_t>(v.size());
    out.write(reinterpret_cast<const char*>(&n), sizeof(n));
    if (n) out.write(reinterpret_cast<const char*>(v.data()), n * sizeof(T));
}

template<typename T>
void read_vec(std::istream& in, std::vector<T>& v) {
    uint32_t n = 0;
    in.read(reinterpret_cast<char*>(&n), sizeof(n));
    v.resize(n);
    if (n) in.read(reinterpret_cast<char*>(v.data()), n * sizeof(T));
}

void write_strings(std::ostream& out, const std::vector<std::string>& v) {
    uint32_t n = static_cast<uint32_t>(v.size());
    out.write(reinterpret_cast<const char*>(&n), sizeof(n));
    for (const auto& s : v) {
        uint32_t len = static_cast<uint32_t>(s.size());
        out.write(reinterpret_cast<const char*>(&len), sizeof(len));
        out.write(s.data(), len);
    }
}

void read_strings(std::istream& in, std::vector<std::string>& v) {
    uint32_t n = 0;
    in.read(reinterpret_cast<char*>(&n), sizeof(n));
    v.resize(n);
    for (auto& s : v) {
        uint32_t len = 0;
        in.read(reinterpret_cast<char*>(&len), sizeof(len));
        s.resize(len);
        if (len) in.read(s.data(), len);
    }
}

} // namespace

void Graph::save(const std::filesystem::path& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("Cannot open graph cache for writing: " + path.string());

    out.write(kMagic, 4);
    out.write(reinterpret_cast<const char*>(&kVersion), sizeof(kVersion));

    uint32_t N = num_nodes(), E = num_edges();
    out.write(reinterpret_cast<const char*>(&N), sizeof(N));
    out.write(reinterpret_cast<const char*>(&E), sizeof(E));

    write_vec(out, row_ptr);
    write_vec(out, col_idx);
    write_vec(out, edges);
    write_vec(out, nodes);
    write_vec(out, geom_ptr);
    write_vec(out, geom_coords);
    write_vec(out, osm_node_ids);
    write_vec(out, way_ids);
    write_strings(out, way_names);
    write_vec(out, signal_schedules);

    if (!out)
        throw std::runtime_error("Write error on graph cache: " + path.string());
}

Graph Graph::load(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("Cannot open graph cache: " + path.string());

    char magic[4];
    in.read(magic, 4);
    if (std::memcmp(magic, kMagic, 4) != 0)
        throw std::runtime_error("Invalid graph cache (bad magic): " + path.string());

    uint32_t ver = 0;
    in.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    if (ver != kVersion)
        throw std::runtime_error("Graph cache version mismatch (expected " +
                                  std::to_string(kVersion) + ", got " +
                                  std::to_string(ver) + "): " + path.string());

    uint32_t N = 0, E = 0;
    in.read(reinterpret_cast<char*>(&N), sizeof(N));
    in.read(reinterpret_cast<char*>(&E), sizeof(E));

    Graph g;
    read_vec(in, g.row_ptr);
    read_vec(in, g.col_idx);
    read_vec(in, g.edges);
    read_vec(in, g.nodes);
    read_vec(in, g.geom_ptr);
    read_vec(in, g.geom_coords);
    read_vec(in, g.osm_node_ids);
    read_vec(in, g.way_ids);
    read_strings(in, g.way_names);
    read_vec(in, g.signal_schedules);

    if (!in)
        throw std::runtime_error("Read error on graph cache: " + path.string());
    if (g.num_nodes() != N || g.num_edges() != E)
        throw std::runtime_error("Graph cache corrupted: node/edge count mismatch");

    return g;
}

} // namespace nomad
