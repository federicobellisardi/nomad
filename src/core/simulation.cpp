#include <nomad/core/simulation.hpp>
#include <nomad/traffic/traffic_model.hpp>
#include <nomad/routing/router.hpp>
#include <nomad/routing/astar_router.hpp>
#include <random>
#include <chrono>
#include <nomad/demand/demand_model.hpp>
#include <nomad/output/writer.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <tbb/task_arena.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <spdlog/spdlog.h>

namespace nomad {

namespace {
inline uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// Route plan file (Simulation::write_routes / set_replanning), little endian:
//   "NRTE" u32 version=1 u32 num_edges u32 n_agents
//   per agent: u32 origin u32 dest f64 depart u32 mode u32 len u32 edges[len]
// len = 0 for agents without a route.
constexpr char     kRoutesMagic[4] = {'N', 'R', 'T', 'E'};
constexpr uint32_t kRoutesVersion  = 1;

struct StoredPlan {
    NodeId origin = kInvalidNode, dest = kInvalidNode;
    double depart = 0.0;
    uint32_t mode = 0;
    std::vector<EdgeId> edges;
};

std::vector<StoredPlan> read_routes(const std::filesystem::path& p, std::size_t num_edges) {
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("routes_in: cannot open " + p.string());
    char magic[4]; uint32_t ver = 0, ne = 0, na = 0;
    f.read(magic, 4);
    f.read(reinterpret_cast<char*>(&ver), 4);
    f.read(reinterpret_cast<char*>(&ne), 4);
    f.read(reinterpret_cast<char*>(&na), 4);
    if (!f || std::memcmp(magic, kRoutesMagic, 4) != 0 || ver != kRoutesVersion)
        throw std::runtime_error("routes_in: " + p.string() + " is not a route plan file v1");
    if (ne != num_edges)
        throw std::runtime_error("routes_in: edge count does not match the graph");
    std::vector<StoredPlan> out(na);
    for (auto& sp : out) {
        uint32_t len = 0;
        f.read(reinterpret_cast<char*>(&sp.origin), 4);
        f.read(reinterpret_cast<char*>(&sp.dest), 4);
        f.read(reinterpret_cast<char*>(&sp.depart), 8);
        f.read(reinterpret_cast<char*>(&sp.mode), 4);
        f.read(reinterpret_cast<char*>(&len), 4);
        sp.edges.resize(len);
        if (len) f.read(reinterpret_cast<char*>(sp.edges.data()), 4 * std::streamsize(len));
        if (!f) throw std::runtime_error("routes_in: truncated file " + p.string());
    }
    return out;
}

// A stored route is reusable only if it is a connected path origin -> dest.
bool route_connects(const Graph& g, NodeId origin, NodeId dest, const std::vector<EdgeId>& r) {
    if (r.empty()) return false;
    NodeId at = origin;
    for (EdgeId e : r) {
        if (e >= g.num_edges() || at >= g.num_nodes()) return false;
        auto out = g.out_edges(at);
        if (std::find(out.begin(), out.end(), e) == out.end()) return false;
        at = g.edges[e].target;
    }
    return at == dest;
}
} // namespace

void Simulation::write_routes(const std::filesystem::path& p) const {
    std::ofstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("routes_out: cannot open " + p.string());
    const uint32_t ne = static_cast<uint32_t>(graph_->num_edges());
    const uint32_t na = static_cast<uint32_t>(hot_.size());
    f.write(kRoutesMagic, 4);
    f.write(reinterpret_cast<const char*>(&kRoutesVersion), 4);
    f.write(reinterpret_cast<const char*>(&ne), 4);
    f.write(reinterpret_cast<const char*>(&na), 4);
    for (AgentId a = 0; a < na; ++a) {
        const auto& plan = cold_.plans[a];
        const bool has = plan.activities.size() >= 2;
        const uint32_t o = has ? plan.activities[0].location : kInvalidNode;
        const uint32_t d = has ? plan.activities[1].location : kInvalidNode;
        const double  t = has ? plan.activities[0].start_time : 0.0;
        const uint32_t m = static_cast<uint32_t>(plan.preferred_mode);
        static const std::vector<EdgeId> kEmpty;
        const auto& r = a < routes_.routes.size() ? routes_.routes[a] : kEmpty;
        const uint32_t len = static_cast<uint32_t>(r.size());
        f.write(reinterpret_cast<const char*>(&o), 4);
        f.write(reinterpret_cast<const char*>(&d), 4);
        f.write(reinterpret_cast<const char*>(&t), 8);
        f.write(reinterpret_cast<const char*>(&m), 4);
        f.write(reinterpret_cast<const char*>(&len), 4);
        if (len) f.write(reinterpret_cast<const char*>(r.data()), 4 * std::streamsize(len));
    }
    if (!f) throw std::runtime_error("routes_out: write failed " + p.string());
    spdlog::info("Route plans of {} agents written to {}", na, p.string());
}


struct Simulation::Impl {
    tbb::task_arena arena;
    explicit Impl(int nthreads)
        : arena(nthreads > 0 ? nthreads : tbb::task_arena::automatic) {}
};

Simulation::Simulation(SimulationConfig cfg)
    : cfg_(std::move(cfg)),
      impl_(std::make_unique<Impl>(static_cast<int>(cfg_.num_threads)))
{}

Simulation::~Simulation() = default;

void Simulation::set_graph(std::shared_ptr<Graph> g)               { graph_   = std::move(g); }
void Simulation::set_demand(std::shared_ptr<IDemandModel> d)       { demand_  = std::move(d); }
void Simulation::set_traffic_model(std::unique_ptr<ITrafficModel> m){ traffic_ = std::move(m); }
void Simulation::set_router(std::unique_ptr<IRouter> r)            { router_  = std::move(r); }

void Simulation::add_output_writer(std::unique_ptr<IOutputWriter> w) {
    writers_.push_back(std::move(w));
}

void Simulation::register_hook(EventType type, std::function<void(const Event&)> cb) {
    hooks_[static_cast<uint8_t>(type)].push_back(std::move(cb));
}

void Simulation::fire_hooks(EventType type, const Event& e) {
    auto it = hooks_.find(static_cast<uint8_t>(type));
    if (it == hooks_.end()) return;
    for (const auto& cb : it->second) cb(e);
}

void Simulation::inject_demand() {
    if (!demand_) return;
    std::size_t n = demand_->generate(*graph_, eq_, cold_, routes_);
    hot_.resize(n);
    // hot_.resize() defaults every agent to AgentMode::Car — copy the real
    // per-agent mode from the demand plan so mode-gated logic downstream
    // (traffic model interaction, mode-capped free-flow time) sees it.
    for (AgentId a = 0; a < static_cast<AgentId>(n); ++a)
        hot_.mode[a] = cold_.plans[a].preferred_mode;
    spdlog::info("Simulation: {} agents injected", n);

    if (!router_ || n == 0) return;

    // Build the pre-trip-reroute departure-order index once, up front (see
    // schedule_pretrip_reroutes()) -- O(n log n), trivial next to the
    // pre-routing pass below. Sorted by each agent's own scheduled
    // departure time, which cold_.plans never mutates afterwards, so a
    // monotonic cursor can walk this exactly once as `now` advances instead
    // of re-scanning all agents every reroute cycle.
    if (cfg_.enable_pretrip_reroute) {
        pretrip_depart_order_.clear();
        pretrip_depart_order_.reserve(n);
        for (AgentId a = 0; a < static_cast<AgentId>(n); ++a)
            if (cold_.plans[a].activities.size() >= 2)
                pretrip_depart_order_.push_back(a);
        std::sort(pretrip_depart_order_.begin(), pretrip_depart_order_.end(),
            [&](AgentId x, AgentId y) {
                return cold_.plans[x].activities[0].start_time
                     < cold_.plans[y].activities[0].start_time;
            });
        pretrip_cursor_ = 0;
    }

    // ── Pre-routing: compute all routes before the simulation loop starts ─────
    // Parallel phase: AStarRouter.route() is thread-safe (per-thread TLS).
    // Sequential phase: push results into RouteStore (not thread-safe).
    spdlog::info("Pre-routing {} agents on {} threads...", n,
        tbb::this_task_arena::max_concurrency());

    std::vector<Route> route_results(n);
    auto t0 = std::chrono::steady_clock::now();

    // Per-edge stochastic pre-routing: each agent gets a deterministic
    // per-edge cost perturbation derived from hash(agent_id, edge_id).
    // Unlike per-class perturbation (which avoids entire road classes causing
    // large detours), per-edge noise distributes agents across nearby junction
    // alternatives while keeping route lengths similar to the CH optimum.
    // sigma = 0.08: each edge costs ±8% of free-flow → agents diverge at
    // junctions where multiple paths are within 8% of each other in travel time.
    const float sigma = cfg_.route_randomization_sigma;
    // (seed - 42) so the default seed leaves the historical constant unchanged.
    const uint32_t route_seed_base = 0xCA7F00Du ^ ((cfg_.seed - 42u) * 0x85ebca6bu);
    std::unique_ptr<AStarRouter> stoch_router;
    const TdCostTable* td = td_in_.get();
    if (td) spdlog::info("Pre-routing cars on time-dependent link costs ({} bins of {:.0f}s)",
                         td->num_bins(), td->bin_s());
    if (sigma > 0.0f || td) {
        AStarRouter::Config stoch_cfg;
        stoch_cfg.use_traffic_costs = false;
        stoch_cfg.max_speed_ms      = 33.3f;
        stoch_cfg.walk_speed_ms     = cfg_.walk_speed_ms;
        stoch_cfg.bike_speed_ms     = cfg_.bike_speed_ms;
        stoch_router = std::make_unique<AStarRouter>(*graph_, nullptr, stoch_cfg);
        if (sigma > 0.0f)
            spdlog::info("Per-edge stochastic pre-routing enabled (sigma={:.2f})", sigma);
    }

    // ── Partial replanning: keep the stored plans of agents not drawn ─────────
    std::vector<StoredPlan> stored;
    std::vector<uint8_t>    keep;
    n_plan_kept_ = n_plan_rerouted_ = n_plan_mismatch_ = 0;
    if (!routes_in_.empty()) {
        stored = read_routes(routes_in_, graph_->num_edges());
        keep.assign(n, 0);
        for (AgentId a = 0; a < static_cast<AgentId>(n); ++a) {
            const auto& plan = cold_.plans[a];
            if (plan.activities.size() < 2) continue;
            const bool match = a < stored.size()
                && stored[a].origin == plan.activities[0].location
                && stored[a].dest   == plan.activities[1].location
                && std::abs(stored[a].depart - plan.activities[0].start_time) < 1e-6
                && stored[a].mode   == static_cast<uint32_t>(plan.preferred_mode)
                && route_connects(*graph_, stored[a].origin, stored[a].dest, stored[a].edges);
            if (!match) { ++n_plan_mismatch_; continue; }
            uint64_t st = (static_cast<uint64_t>(replan_seed_) << 32) ^ a ^ 0x5EED9A7A11ull;
            const double u = (splitmix64(st) >> 11) * (1.0 / 9007199254740992.0);
            if (u >= replan_fraction_) { keep[a] = 1; ++n_plan_kept_; }
            else ++n_plan_rerouted_;
        }
        spdlog::info("Replanning (fraction {:.3f}, seed {}): {} plans kept, {} re-routed, "
                     "{} without a matching stored plan (re-routed)",
                     replan_fraction_, replan_seed_, n_plan_kept_, n_plan_rerouted_, n_plan_mismatch_);
        if (n_plan_mismatch_ > n / 100)
            spdlog::warn("  {} agents did not match their stored plan: is the demand "
                         "(seed, scale, OD file) identical to the previous iteration?",
                         n_plan_mismatch_);
    }

    std::atomic<std::size_t> progress{0};
    std::atomic<uint64_t>    n_stoch{0};
    std::atomic<uint64_t>    n_fallback{0};
    impl_->arena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<AgentId>(0, static_cast<AgentId>(n), 256),
            [&](const tbb::blocked_range<AgentId>& rng) {
                for (AgentId a = rng.begin(); a < rng.end(); ++a) {
                    const auto& plan = cold_.plans[a];
                    if (plan.activities.size() < 2) continue;
                    if (!keep.empty() && keep[a]) {
                        route_results[a].edges    = std::move(stored[a].edges);
                        route_results[a].is_valid = true;
                        continue;
                    }
                    RoutingRequest req{
                        plan.activities[0].location,
                        plan.activities[1].location,
                        plan.activities[0].start_time,
                        plan.preferred_mode
                    };

                    if (stoch_router) {
                        uint32_t seed = route_seed_base ^ (static_cast<uint32_t>(a) * 2654435761u);
                        Route r = stoch_router->route_stochastic(
                            req.origin, req.destination, req.mode, seed, sigma,
                            td, req.departure_time);

                        // Reject extreme detours: > 2× crow-fly AND > 5 km.
                        // Falls back to CH so bad hashes never lose an agent.
                        float haversine_m = graph_->haversine(req.origin, req.destination);
                        // Not applied with time-dependent costs: a detour
                        // around a queue is then a legitimate choice.
                        bool too_long = !td
                                        && r.estimated_dist_m > 2.0f * haversine_m
                                        && r.estimated_dist_m > 5000.0f;
                        if (r.is_valid && !r.edges.empty() && !too_long) {
                            route_results[a] = std::move(r);
                            ++n_stoch;
                            continue;
                        }
                        ++n_fallback;
                    }

                    route_results[a] = router_->route(req);
                }
                std::size_t done = progress.fetch_add(rng.size()) + rng.size();
                if (done % 20000 < 256) {
                    double elapsed = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    spdlog::info("  Pre-routed {}/{} — {:.0f} routes/s",
                        done, n, done / std::max(elapsed, 0.001));
                }
            });
    });
    if (sigma > 0.0f)
        spdlog::info("  Stochastic routes: {}/{} ({:.1f}%)  CH fallback: {}",
                      n_stoch.load(), n,
                      100.0 * n_stoch.load() / std::max(n, std::size_t{1}),
                      n_fallback.load());

    std::size_t routed = 0, failed = 0;
    for (AgentId a = 0; a < static_cast<AgentId>(n); ++a) {
        if (route_results[a].is_valid && !route_results[a].edges.empty()) {
            routes_.push_route(a, route_results[a].edges);
            ++routed;
        } else {
            ++failed;
        }
    }

    { std::vector<StoredPlan>().swap(stored); }
    if (!routes_out_.empty()) write_routes(routes_out_);

    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    spdlog::info("Pre-routing done: {}/{} routed, {} failed (no route) in {:.1f}s ({:.0f} routes/s)",
        routed, n, failed, elapsed, routed / std::max(elapsed, 0.001));
    if (failed > 0)
        spdlog::warn("  {} agents ({:.1f}%) have no valid route — "
                     "they will be silently marked Arrived at departure. "
                     "Check OD connectivity (excluded road classes may have isolated nodes).",
                     failed, 100.0 * failed / std::max(n, std::size_t{1}));

    // ── Per-agent free-flow route time (stored for teleport mechanism) ─────────
    if (graph_) {
        std::size_t beyond_window = 0, will_finish = 0;
        float max_trip_s = 0, sum_trip_s = 0;
        uint32_t n_sampled = 0;
        for (AgentId a = 0; a < static_cast<AgentId>(n); ++a) {
            auto edges = routes_.get_route(a);
            if (edges.empty()) continue;
            AgentMode mode = hot_.mode[a];
            float tt = 0;
            for (EdgeId eid : edges) {
                if (eid < graph_->num_edges())
                    tt += graph_->mode_free_flow_time(eid, mode, cfg_.walk_speed_ms, cfg_.bike_speed_ms);
            }
            hot_.freeflow_route_s[a] = tt;   // store for teleport
            SimTime t_dep = cold_.plans[a].activities[0].start_time;
            if (t_dep + tt > cfg_.end_time) ++beyond_window;
            else ++will_finish;
            max_trip_s = std::max(max_trip_s, tt);
            sum_trip_s += tt;
            ++n_sampled;
        }
        float avg = n_sampled > 0 ? sum_trip_s / n_sampled : 0;
        spdlog::info("Trip durations: avg={:.0f}s ({:.1f}min) max={:.0f}s | "
                     "will_finish={} beyond_sim_window={}",
                     avg, avg / 60.0f, max_trip_s, will_finish, beyond_window);
    }
}

void Simulation::handle_event(const Event& e) {
    ++events_processed_;
    switch (e.type) {
    case EventType::AgentDepart:         ++n_depart_;  handle_depart(e);           break;
    // AgentEnterLink is no longer pushed as a queued event by anything (see
    // handle_depart()/handle_exit_link(): both enter synchronously via
    // enter_edge_now() to avoid a same-batch staleness race on the
    // destination edge's occupancy) -- kept in EventType for the hook API
    // (fire_hooks(EventType::AgentEnterLink, ...) is still called manually
    // from both), just never dispatched through this switch.
    case EventType::AgentExitLink:       ++n_exit_;    handle_exit_link(e);        break;
    case EventType::AgentArriveActivity: ++n_arrive_;  handle_arrive_activity(e);  break;
    case EventType::AgentReroute:        handle_reroute(e);          break;
    case EventType::SignalPhaseChange:   handle_signal_change(e);    break;
    case EventType::NodeStep:            handle_node_step(e);        break;
    default: break;
    }
    fire_hooks(e.type, e);
}

void Simulation::handle_depart(const Event& e) {
    AgentId a = e.agent;
    if (a >= hot_.size()) return;

    // Try pre-computed route first (set by inject_demand's pre-routing phase)
    auto pre_edges = routes_.get_route(a);
    if (!pre_edges.empty()) {
        // Gate a trip's *first* edge: there is no preceding on_exit() call for
        // a trip origin (no "previous" edge), so on_enter() would otherwise be
        // reachable with unbounded occupancy — every subsequent transition is
        // already gated by on_exit()'s spillback check on `next` below.
        // Reject-and-retry 1s later, same convention as handle_exit_link()'s
        // spillback rejection. hot_.state[a] is left untouched (stays
        // AgentState::Waiting) so a rejected departure doesn't falsely
        // report as OnLink.
        //
        // Entered SYNCHRONOUSLY here (enter_edge_now), not via a queued
        // AgentEnterLink event: within a single drain_until() batch, several
        // agents' AgentDepart events are processed back-to-back, and any
        // event newly pushed to eq_ mid-batch (like a queued AgentEnterLink)
        // only becomes visible on the NEXT drain — so has_capacity() would
        // see stale (pre-increment) occupancy for every agent already
        // processed earlier in the SAME batch, and the gate would silently
        // do nothing whenever multiple departures land in one batch (verified
        // empirically: a burst of simultaneous departures blew straight past
        // storage_veh before this synchronous-entry fix). Entering
        // immediately makes the occupancy increment visible to the very next
        // iteration of this same batch's dispatch loop.
        // n_enter_/fire_hooks are done here explicitly since this path never
        // dispatches a genuine AgentEnterLink event (handle_event's switch,
        // which normally does both, is bypassed for this transition).
        if (traffic_ && hot_.mode[a] == AgentMode::Car &&
            !traffic_->has_capacity_departure_path(pre_edges[0],
                                        std::span<const EdgeId>(pre_edges).subspan(1))) {
            ++n_depart_rejected_;
            eq_.push({e.time + 1.0, a, e.payload, EventType::AgentDepart, {}});
            return;
        }
        hot_.route_pos[a] = 0;
        enter_edge_now(a, pre_edges[0], e.time);
        ++n_enter_;
        fire_hooks(EventType::AgentEnterLink, {e.time, a, pre_edges[0], EventType::AgentEnterLink, {}});
        return;
    }

    // Fall-back: on-demand routing (used when pre-routing was skipped)
    const auto& plan = cold_.plans[a];
    if (plan.activities.size() < 2 || !router_) {
        hot_.state[a] = AgentState::Arrived;
        return;
    }
    NodeId origin = plan.activities[0].location;
    NodeId dest   = plan.activities[1].location;

    RoutingRequest req{origin, dest, e.time, hot_.mode[a]};
    Route route = router_->route(req);
    if (!route.is_valid || route.edges.empty()) {
        hot_.state[a] = AgentState::Arrived;
        return;
    }

    if (traffic_ && hot_.mode[a] == AgentMode::Car &&
        !traffic_->has_capacity_departure_path(route.edges[0],
                                    std::span<const EdgeId>(route.edges).subspan(1))) {
        ++n_depart_rejected_;
        eq_.push({e.time + 1.0, a, e.payload, EventType::AgentDepart, {}});
        return;
    }

    routes_.push_route(a, route.edges);
    hot_.route_pos[a] = 0;
    enter_edge_now(a, route.edges[0], e.time);
    ++n_enter_;
    fire_hooks(EventType::AgentEnterLink, {e.time, a, route.edges[0], EventType::AgentEnterLink, {}});
}

std::span<const EdgeId> Simulation::route_after(AgentId a) const {
    if (a >= routes_.routes.size()) return {};
    const auto& r = routes_.routes[a];
    const std::size_t k = static_cast<std::size_t>(hot_.route_pos[a]) + 1;
    return k < r.size() ? std::span<const EdgeId>(r).subspan(k) : std::span<const EdgeId>{};
}

EdgeId Simulation::next_edge_after_current(AgentId a, std::size_t ahead) const {
    if (a >= routes_.routes.size()) return kInvalidEdge;
    const auto& r = routes_.routes[a];
    std::size_t k = static_cast<std::size_t>(hot_.route_pos[a]) + ahead;
    return k < r.size() ? r[k] : kInvalidEdge;
}

void Simulation::enter_edge_now(AgentId a, EdgeId eid, SimTime t) {
    hot_.current_edge[a] = eid;
    hot_.enter_time[a]   = t;
    hot_.state[a]        = AgentState::OnLink;

    // Only Car agents interact with the (car-calibrated) traffic model —
    // walk/bike travel free-flow at their own mode-capped speed, no
    // congestion interaction (no pedestrian/bike congestion model exists).
    SimTime exit_time;
    if (traffic_ && hot_.mode[a] == AgentMode::Car) {
        exit_time = traffic_->on_enter(eid, a, t);
        // QNode: no overtaking on a link. A car reaches the end of the link
        // (and its FIFO buffer) after the car that entered before it; the
        // 1 ms step keeps equal times from being reordered by the queue.
        if (qnode_ && eid < qlast_exit_.size()) {
            exit_time = std::max(exit_time, qlast_exit_[eid] + 1e-3);
            qlast_exit_[eid] = exit_time;
        }
        traffic_->record_enter(eid, t);
        traffic_->assign_movement_path(eid, a, route_after(a));
    } else {
        exit_time = t + graph_->mode_free_flow_time(
            eid, hot_.mode[a], cfg_.walk_speed_ms, cfg_.bike_speed_ms);
    }

    hot_.scheduled_exit[a] = exit_time;
    eq_.push({exit_time, a, eid, EventType::AgentExitLink, {}});
}

void Simulation::handle_exit_link(const Event& e) {
    AgentId a   = e.agent;
    EdgeId  eid = static_cast<EdgeId>(e.payload);
    if (a >= hot_.size()) return;
    if (hot_.state[a] == AgentState::Arrived) return;  // already teleported; force_remove already called
    if (!graph_ || eid >= graph_->num_edges()) {
        hot_.state[a] = AgentState::Arrived;
        return;
    }

    // QNode node model: the car has reached the end of its link; it waits in
    // the link's FIFO buffer and the downstream node releases it.
    if (qnode_ && traffic_ && hot_.mode[a] == AgentMode::Car) {
        qbuf_[eid].push_back(a);
        schedule_node_step(graph_->edges[eid].target, e.time);
        return;
    }

    // Peek the next edge (without committing route_pos) so the traffic model
    // can check downstream spillback before releasing the agent.
    uint16_t pos  = hot_.route_pos[a] + 1;
    auto     route = routes_.get_route(a);
    EdgeId   next  = (pos < route.size()) ? route[pos] : kInvalidEdge;

    bool can_exit = true;
    if (traffic_ && hot_.mode[a] == AgentMode::Car)
        can_exit = traffic_->on_exit_path(eid, std::span<const EdgeId>(route).subspan(pos), a, e.time);

    if (!can_exit) {
        eq_.push({e.time + 1.0, a, eid, EventType::AgentExitLink, {}});
        return;
    }
    complete_exit(a, eid, next, e.time);
}

void Simulation::complete_exit(AgentId a, EdgeId eid, EdgeId next, SimTime t) {
    const Event e{t, a, eid, EventType::AgentExitLink, {}};
    const uint16_t pos = hot_.route_pos[a] + 1;
    if (traffic_ && hot_.mode[a] == AgentMode::Car) traffic_->record_exit(eid, e.time);
    if (td_rec_ && hot_.mode[a] == AgentMode::Car)
        td_rec_->record(eid, hot_.enter_time[a], e.time);

    hot_.route_pos[a] = pos;

    if (next == kInvalidEdge) {
        hot_.state[a] = AgentState::AtActivity;
        eq_.push({e.time, a, eid, EventType::AgentArriveActivity, {}});
        return;
    }

    // Entered SYNCHRONOUSLY (enter_edge_now), not via a queued AgentEnterLink
    // event — the exact same-batch staleness bug fixed in handle_depart()
    // also applies here: if several agents from DIFFERENT upstream edges exit
    // onto this SAME `next` within one drain_until() batch, on_exit()'s
    // spillback check on `next` (above) reads its occupancy BEFORE any of
    // them actually increment it (a queued AgentEnterLink only lands on the
    // NEXT batch) — so every one of them passes the check against the same
    // stale value, and `next` can end up over storage_veh despite each
    // individual on_exit() call looking correct in isolation. Confirmed
    // empirically: this was the source of a residual Via de Cintura
    // over-capacity reading (126%) that survived the handle_depart() fix
    // alone. Entering here immediately makes the increment visible to the
    // very next iteration of this same batch's dispatch loop.
    enter_edge_now(a, next, e.time);
    ++n_enter_;
    fire_hooks(EventType::AgentEnterLink, {e.time, a, next, EventType::AgentEnterLink, {}});
}

// ── QNode node model (cfg_.node_model == "qnode") ──────────────────────────────
// After MATSim's QSim QNode. A car whose earliest exit time has passed waits in
// the FIFO buffer of its link (handle_exit_link). Once per simulated second a
// node with waiting cars is served: its in-links with a non-empty buffer are
// visited in random order, drawn without replacement with probability
// proportional to link capacity; each in-link releases cars in FIFO order
// while the traffic model admits them (on_exit_to: downstream storage,
// discharge cap, stuck-time squeeze). Earlier-served in-links take the
// downstream space first, so over time a congested merge is shared in
// proportion to approach capacity. With fifo_per_movement() a blocked
// movement does not block cars heading to another next link.

void Simulation::init_qnode() {
    if (cfg_.node_model != "qnode" && cfg_.node_model != "agent")
        throw std::runtime_error("unknown node_model: " + cfg_.node_model);
    qnode_ = cfg_.node_model == "qnode";
    if (!qnode_ || !graph_) return;
    const uint32_t N = graph_->num_nodes(), E = graph_->num_edges();
    qbuf_.assign(E, {});
    qlast_exit_.assign(E, -1e18);
    qnode_next_.assign(N, -1.0);
    qin_ptr_.assign(N + 1, 0);
    for (uint32_t e = 0; e < E; ++e)
        if (graph_->edges[e].target < N) ++qin_ptr_[graph_->edges[e].target + 1];
    for (uint32_t u = 0; u < N; ++u) qin_ptr_[u + 1] += qin_ptr_[u];
    qin_idx_.assign(E, kInvalidEdge);
    std::vector<uint32_t> fill(qin_ptr_.begin(), qin_ptr_.end() - 1);
    for (uint32_t e = 0; e < E; ++e) {
        NodeId t = graph_->edges[e].target;
        if (t < N) qin_idx_[fill[t]++] = e;
    }
    spdlog::info("Node model: qnode (capacity-weighted in-link service, FIFO {})",
                 traffic_ && traffic_->fifo_per_movement() ? "per movement" : "per link");
}

void Simulation::schedule_node_step(NodeId n, SimTime t) {
    if (n >= qnode_next_.size() || qnode_next_[n] >= 0.0) return;   // already pending
    const SimTime ts = std::floor(t) + 1.0;   // next whole second
    qnode_next_[n] = ts;
    eq_.push({ts, kNoAgent, n, EventType::NodeStep, {}});
}


void Simulation::handle_node_step(const Event& e) {
    const NodeId n = e.payload;
    const SimTime t = e.time;
    if (n >= qnode_next_.size()) return;
    qnode_next_[n] = -1.0;

    // in-links with waiting cars, and their capacities
    std::vector<std::pair<EdgeId, float>> act;
    double total = 0.0;
    for (uint32_t k = qin_ptr_[n]; k < qin_ptr_[n + 1]; ++k) {
        EdgeId ein = qin_idx_[k];
        if (qbuf_[ein].empty()) continue;
        float c = std::max(1.0f, graph_->edges[ein].capacity);
        act.emplace_back(ein, c);
        total += c;
    }
    // reproducible stream per (seed, node, second)
    uint64_t st = (static_cast<uint64_t>(cfg_.seed) << 32) ^ (static_cast<uint64_t>(n) * 0x9E3779B1ull)
                  ^ (static_cast<uint64_t>(t) * 0xC2B2AE3D27D4EB4Full);
    const bool per_mv = traffic_ && traffic_->fifo_per_movement();
    bool remaining = false;

    while (!act.empty()) {
        // draw an in-link with probability proportional to capacity
        const double u = (splitmix64(st) >> 11) * (1.0 / 9007199254740992.0) * total;
        std::size_t pick = act.size() - 1;
        double acc = 0.0;
        for (std::size_t i = 0; i < act.size(); ++i) {
            acc += act[i].second;
            if (u < acc) { pick = i; break; }
        }
        const EdgeId ein = act[pick].first;
        total -= act[pick].second;
        act[pick] = act.back();
        act.pop_back();

        auto& buf = qbuf_[ein];
        // per-movement FIFO: movements (fifo_group: next link, or lane group
        // with lookahead) blocked this step
        std::vector<EdgeId> blocked_mv;
        for (std::size_t i = 0; i < buf.size();) {
            const AgentId a = buf[i];
            // teleported or otherwise gone from this link: drop the stale entry
            if (a >= hot_.size() || hot_.state[a] != AgentState::OnLink || hot_.current_edge[a] != ein) {
                buf.erase(buf.begin() + static_cast<std::ptrdiff_t>(i));
                continue;
            }
            const std::span<const EdgeId> after = route_after(a);
            const EdgeId next = after.empty() ? kInvalidEdge : after[0];
            const EdgeId mv = per_mv ? traffic_->fifo_group(ein, after) : kInvalidEdge;
            if (per_mv && std::find(blocked_mv.begin(), blocked_mv.end(), mv) != blocked_mv.end()) {
                ++i; continue;
            }
            if (traffic_->on_exit_path(ein, after, a, t)) {
                buf.erase(buf.begin() + static_cast<std::ptrdiff_t>(i));
                complete_exit(a, ein, next, t);
                continue;
            }
            // Not admitted. Plain FIFO: the whole link waits. Per movement:
            // only this movement waits, unless the block is on the link's own
            // discharge (downstream has room), which stops the link.
            if (!per_mv || next == kInvalidEdge ||
                traffic_->has_capacity_path(next, after.subspan(1))) break;
            blocked_mv.push_back(mv);
            ++i;
        }
        if (!buf.empty()) remaining = true;
    }
    if (remaining) schedule_node_step(n, t);
}

void Simulation::handle_arrive_activity(const Event& e) {
    AgentId a = e.agent;
    if (a >= hot_.size()) return;
    hot_.state[a] = AgentState::Arrived;
}

void Simulation::handle_reroute(const Event& e) {
    AgentId a = e.agent;
    if (a >= hot_.size() || !graph_) return;
    if (hot_.state[a] != AgentState::OnLink) return;

    const auto& plan = cold_.plans[a];
    if (plan.activities.size() < 2) return;

    NodeId dest = plan.activities.back().location;
    EdgeId cur  = hot_.current_edge[a];
    if (cur >= graph_->num_edges()) return;
    NodeId from = graph_->edges[cur].target;

    // Use traffic-aware A* for rerouting; fall back to primary router if unavailable.
    IRouter* r_ptr = reroute_router_ ? static_cast<IRouter*>(reroute_router_.get())
                                     : router_.get();
    if (!r_ptr) return;

    RoutingRequest req{from, dest, e.time, hot_.mode[a]};
    // Stop rerouting agents that have already been rerouted too many times —
    // they are likely in a routing loop and should complete their current path.
    if (cfg_.max_reroutes > 0 && hot_.reroute_count[a] >= cfg_.max_reroutes)
        return;

    Route r = r_ptr->route(req);
    if (r.is_valid && !r.edges.empty()) {
        // route_pos indexes the edge the agent is on; the new route starts
        // at that edge's target, so it replaces everything AFTER it.
        routes_.replace_suffix(a, static_cast<uint16_t>(hot_.route_pos[a] + 1), r.edges);
        if (traffic_ && hot_.mode[a] == AgentMode::Car && hot_.state[a] == AgentState::OnLink)
            traffic_->assign_movement_path(hot_.current_edge[a], a, route_after(a));
        // Keep freeflow_route_s in sync with the rerouted path so the stuck
        // threshold (ratio × ff) is never exceeded by BPR-congested agents on
        // a longer but legitimate detour.
        auto full = routes_.get_route(a);
        float ff = 0;
        for (EdgeId eid : full)
            if (eid < graph_->num_edges())
                ff += graph_->mode_free_flow_time(eid, hot_.mode[a], cfg_.walk_speed_ms, cfg_.bike_speed_ms);
        hot_.freeflow_route_s[a] = ff;
        ++hot_.reroute_count[a];
    }
}

void Simulation::handle_signal_change(const Event&) {}

void Simulation::teleport_stuck_agents(SimTime now) {
    if (cfg_.stuck_threshold_ratio <= 0.0f && cfg_.stuck_max_hours <= 0.0f) return;
    if (now - last_teleport_t_ < cfg_.teleport_interval_s) return;
    last_teleport_t_ = now;

    // Hard cap in seconds (0 = disabled).
    const double hard_cap_s = cfg_.stuck_max_hours > 0.0f
                              ? static_cast<double>(cfg_.stuck_max_hours) * 3600.0
                              : std::numeric_limits<double>::infinity();

    const uint32_t N = static_cast<uint32_t>(hot_.size());
    uint64_t batch = 0;
    for (AgentId a = 0; a < N; ++a) {
        if (hot_.state[a] == AgentState::Waiting) {
            // A departure can now be rejected indefinitely by
            // ITrafficModel::has_capacity() if its origin edge never frees
            // up (see handle_depart()) — unlike OnLink agents there is no
            // route-based free-flow time to scale a ratio threshold from
            // (the agent hasn't started moving), so this uses the hard cap
            // only, same stuck_max_hours ceiling as below.
            if (hard_cap_s == std::numeric_limits<double>::infinity()) continue;
            double depart = cold_.plans[a].activities.empty()
                          ? cfg_.start_time
                          : cold_.plans[a].activities[0].start_time;
            if (now - depart <= hard_cap_s) continue;
            hot_.state[a] = AgentState::Arrived;
            ++batch;
            continue;
        }
        if (hot_.state[a] != AgentState::OnLink) continue;
        float ff = hot_.freeflow_route_s[a];
        if (ff <= 0.0f) continue;

        double depart = cold_.plans[a].activities.empty()
                      ? cfg_.start_time
                      : cold_.plans[a].activities[0].start_time;
        double elapsed = now - depart;

        // Threshold: ratio × current route ff, capped by the hard ceiling.
        // After rerouting, ff_route can grow large enough that ratio×ff exceeds
        // the simulation window — the hard cap ensures those agents are still
        // released within stuck_max_hours regardless of route length.
        double ratio_thresh = cfg_.stuck_threshold_ratio > 0.0f
                              ? cfg_.stuck_threshold_ratio * static_cast<double>(ff)
                              : std::numeric_limits<double>::infinity();
        double threshold = std::min(ratio_thresh, hard_cap_s);

        if (elapsed <= threshold) continue;

        // Teleport: remove from link, mark arrived
        EdgeId eid = hot_.current_edge[a];
        if (eid < graph_->num_edges() && traffic_ && hot_.mode[a] == AgentMode::Car) {
            traffic_->force_remove(eid, a);
            traffic_->record_exit(eid, now);
            if (td_rec_) td_rec_->record(eid, hot_.enter_time[a], now);
        }

        uint32_t rc = std::min<uint32_t>(hot_.reroute_count[a], teleported_by_reroute_count_.size() - 1);
        ++teleported_by_reroute_count_[rc];
        if (eid < graph_->num_edges())
            ++teleported_by_class_[graph_->edges[eid].road_class];

        hot_.state[a] = AgentState::Arrived;
        ++batch;
    }

    if (batch > 0) {
        n_teleported_ += batch;
        spdlog::debug("teleport t={:.0f}: {} agents teleported (ratio>{:.0f}×ff)",
                      now, batch, cfg_.stuck_threshold_ratio);
    }
}

void Simulation::flush_traffic_state() {
    SimTime t = current_time_.load();
    if (traffic_) traffic_->update(t);
    for (auto& w : writers_)
        if (graph_ && traffic_) w->on_snapshot(t, hot_, *graph_, *traffic_);
}

void Simulation::schedule_reroutes() {
    if (!traffic_ || !graph_) return;

    SimTime now = current_time_.load();
    if (now - last_reroute_t_ < cfg_.reroute_interval_s) return;
    last_reroute_t_ = now;

    const auto states = traffic_->link_states();
    const uint32_t E = graph_->num_edges();

    // 1. Identify congested edges via the smoothed congestion_ema (tt/ff
    //    averaged over several sync windows), not the instantaneous
    //    occupancy/travel_time_s. Short, fast-clearing edges (e.g. motorway
    //    junction connectors) can be genuinely congested most of the time yet
    //    show occ==0 at the exact instant of this periodic scan — the EMA
    //    catches that pattern where a single snapshot would miss it.
    std::vector<EdgeId> congested_vec;
    for (uint32_t eid = 0; eid < E; ++eid) {
        float ema = states[eid].congestion_ema.load(std::memory_order_relaxed);
        if (ema > 1.0f + cfg_.reroute_thresh)
            congested_vec.push_back(eid);
    }
    std::unordered_set<EdgeId> congested_set(congested_vec.begin(), congested_vec.end());

    if (congested_vec.empty()) return;

    // Update CH cost table so future departures see updated costs.
    if (router_) router_->update_costs(congested_vec);

    if (!reroute_router_ && !router_) return;

    // 2. Collect agents whose upcoming route crosses a congested link, and
    //    build their reroute request right here (cheap array lookups) --
    //    the expensive part (the actual route() call) happens in parallel
    //    inside apply_reroute_batch(). Skip agents that have already been
    //    rerouted max_reroutes times — repeated rerouting of the same agent
    //    causes route oscillation (routing loop).
    const uint32_t N = static_cast<uint32_t>(hot_.size());
    std::vector<AgentId> to_reroute;
    std::vector<RoutingRequest> reqs;
    to_reroute.reserve(std::min<uint32_t>(N, 65536u));
    reqs.reserve(std::min<uint32_t>(N, 65536u));
    for (AgentId a = 0; a < N; ++a) {
        if (hot_.state[a] != AgentState::OnLink) continue;
        if (cfg_.max_reroutes > 0 && hot_.reroute_count[a] >= cfg_.max_reroutes) continue;
        if (cold_.plans[a].activities.size() < 2) continue;
        auto route    = routes_.get_route(a);
        uint16_t pos  = hot_.route_pos[a];
        // Scan the whole remaining route, not just a handful of edges ahead:
        // a fixed short lookahead only catches congestion right in front of
        // the agent, so anyone whose bottleneck is further down their route
        // (or who never happens to be within a few edges of it exactly when
        // a scan fires) never gets a chance to reroute at all. Capped at 200
        // as a safety bound for pathological routes — real routes post
        // network-simplification are far shorter.
        constexpr uint32_t kMaxScan = 200;
        uint32_t scan_end = static_cast<uint32_t>(
            std::min<std::size_t>(route.size(), pos + 1 + kMaxScan));
        for (uint32_t k = pos + 1; k < scan_end; ++k) {
            if (congested_set.count(route[k])) {
                EdgeId cur = hot_.current_edge[a];
                if (cur >= graph_->num_edges()) break;
                NodeId from = graph_->edges[cur].target;
                NodeId dest = cold_.plans[a].activities.back().location;
                to_reroute.push_back(a);
                reqs.push_back(RoutingRequest{from, dest, now, hot_.mode[a]});
                break;
            }
        }
    }

    std::size_t n_applied = apply_reroute_batch(to_reroute, reqs, /*counts_toward_budget=*/true);

    spdlog::debug("schedule_reroutes t={:.0f}: {} congested links, {}/{} agents rerouted",
                  now, congested_vec.size(), n_applied, to_reroute.size());
}

std::size_t Simulation::apply_reroute_batch(const std::vector<AgentId>& agents,
                                             const std::vector<RoutingRequest>& reqs,
                                             bool counts_toward_budget) {
    if (agents.empty()) return 0;
    IRouter* r_ptr = reroute_router_ ? static_cast<IRouter*>(reroute_router_.get())
                                     : router_.get();
    if (!r_ptr) return 0;

    // 1. Compute new routes in parallel. A* (reroute_router_) is thread-safe:
    //    each thread uses its own workspace. hot_/cold_/graph_ are read-only
    //    here (no events processing concurrently — called only from
    //    run_until()'s between-batch point, not from inside handle_event()).
    std::vector<Route> new_routes(agents.size());
    impl_->arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, agents.size(), 32),
            [&](const tbb::blocked_range<std::size_t>& rng) {
                for (std::size_t i = rng.begin(); i < rng.end(); ++i)
                    new_routes[i] = r_ptr->route(reqs[i]);
            });
    });

    // 2. Apply results sequentially. RouteStore::replace_suffix operates on a
    //    per-agent vector<vector<EdgeId>> (not a shared flat buffer) — the
    //    only real hazard would be concurrent resize() across agents, not
    //    relevant to this strictly sequential write-back loop.
    std::size_t n_applied = 0;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        AgentId a = agents[i];
        if (!new_routes[i].is_valid || new_routes[i].edges.empty()) continue;
        // Mid-trip reroute (OnLink) keeps the edge the agent is on (index
        // route_pos) and replaces what follows it: the new route starts at
        // that edge's target. Replacing from route_pos itself dropped the
        // current edge and made the agent skip the first new edge. Pre-trip
        // reroute (not yet departed) replaces the whole route (from_pos=0) —
        // see schedule_pretrip_reroutes().
        uint16_t from_pos = counts_toward_budget
                                ? static_cast<uint16_t>(hot_.route_pos[a] + 1)
                                : uint16_t{0};
        routes_.replace_suffix(a, from_pos, new_routes[i].edges);
        if (counts_toward_budget && traffic_ && hot_.mode[a] == AgentMode::Car &&
            hot_.state[a] == AgentState::OnLink)
            traffic_->assign_movement_path(hot_.current_edge[a], a, route_after(a));
        // Keep freeflow_route_s in sync with the (re)routed path so the
        // stuck-agent teleport threshold (ratio × freeflow_route_s) is never
        // exceeded by a longer but legitimate detour.
        auto full = routes_.get_route(a);
        float ff = 0;
        for (EdgeId eid : full)
            if (eid < graph_->num_edges())
                ff += graph_->mode_free_flow_time(eid, hot_.mode[a], cfg_.walk_speed_ms, cfg_.bike_speed_ms);
        hot_.freeflow_route_s[a] = ff;
        if (counts_toward_budget) {
            ++hot_.reroute_count[a];
        } else {
            hot_.pretrip_rerouted[a] = 1;
            ++n_pretrip_rerouted_;
        }
        ++n_applied;
    }
    return n_applied;
}

void Simulation::schedule_pretrip_reroutes() {
    if (!cfg_.enable_pretrip_reroute || !traffic_ || !graph_) return;
    if (!reroute_router_ && !router_) return;

    SimTime now = current_time_.load();
    if (now - last_pretrip_reroute_t_ < cfg_.reroute_interval_s) return;
    last_pretrip_reroute_t_ = now;

    // Walk the monotonic cursor through agents in departure-time order,
    // rescuing every still-Waiting car agent whose scheduled departure falls
    // before the next scan (now + reroute_interval_s) with a fresh,
    // congestion-aware route — before they ever depart on the stale
    // free-flow-only route computed at pre-routing time. Unconditional (no
    // congested-edge filter, unlike the OnLink path in schedule_reroutes()):
    // total pretrip route computations over a full day are the same order of
    // magnitude as the existing pre-routing pass either way, so filtering
    // buys little while risking under-serving exactly the agents this exists
    // to help. Always advance the cursor, even for an agent that turns out
    // not to be Waiting/car anymore (e.g. rejected-and-retried past its own
    // window is not re-caught here — a documented v1 simplification, see the
    // plan file) — every agent is visited exactly once.
    std::vector<AgentId> to_reroute;
    std::vector<RoutingRequest> reqs;
    SimTime horizon = now + cfg_.reroute_interval_s;
    while (pretrip_cursor_ < pretrip_depart_order_.size()) {
        AgentId a = pretrip_depart_order_[pretrip_cursor_];
        SimTime dep = cold_.plans[a].activities[0].start_time;
        if (dep >= horizon) break;
        if (hot_.state[a] == AgentState::Waiting && hot_.mode[a] == AgentMode::Car) {
            NodeId origin = cold_.plans[a].activities[0].location;
            NodeId dest   = cold_.plans[a].activities.back().location;
            to_reroute.push_back(a);
            reqs.push_back(RoutingRequest{origin, dest, now, hot_.mode[a]});
        }
        ++pretrip_cursor_;
    }

    std::size_t n_applied = apply_reroute_batch(to_reroute, reqs, /*counts_toward_budget=*/false);

    spdlog::debug("schedule_pretrip_reroutes t={:.0f}: {}/{} waiting agents rerouted",
                  now, n_applied, to_reroute.size());
}

void Simulation::run() { run_until(cfg_.end_time); }

void Simulation::run_until(SimTime until) {
    if (!graph_) throw std::runtime_error("Simulation::run_until: graph not set");

    // Build the traffic-aware A* router used for en-route rerouting.
    if (graph_ && traffic_) traffic_->init_edge_counters(graph_->num_edges());

    // Created here so it sees the already-set traffic_ model.
    if (!reroute_router_ && graph_ && traffic_) {
        AStarRouter::Config acfg;
        acfg.use_traffic_costs = true;
        acfg.walk_speed_ms     = cfg_.walk_speed_ms;
        acfg.bike_speed_ms     = cfg_.bike_speed_ms;
        reroute_router_ = std::make_unique<AStarRouter>(*graph_, traffic_.get(), acfg);
        spdlog::info("Reroute router: A* with traffic costs");
    }

    init_qnode();
    inject_demand();
    current_time_.store(cfg_.start_time);

    spdlog::info("Simulation running to {:.0f}s ({} events in queue)",
                  until, eq_.size());

    std::size_t last_log_events = 0;
    auto t_start = std::chrono::steady_clock::now();

    while (!eq_.empty() && current_time_.load() < until) {
        SimTime wend = current_time_.load() + cfg_.sync_window_s;

        // Log progress every 10000 events
        if (events_processed_.load() - last_log_events >= 10000) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
            double frac = (current_time_.load() - cfg_.start_time) /
                          (until - cfg_.start_time);
            spdlog::info("  t={:.0f}s ({:.0f}%) | events={} | agents_active={} | {:.0f}s elapsed",
                current_time_.load(), frac * 100.0,
                events_processed_.load(), active_agents(), elapsed);
            last_log_events = events_processed_.load();
        }

        process_events_until(wend);

        flush_traffic_state();
        schedule_reroutes();
        schedule_pretrip_reroutes();
        teleport_stuck_agents(wend);
        current_time_.store(wend);
    }

    for (auto& w : writers_) w->flush();

    // ── Final state breakdown ──────────────────────────────────────────────────
    uint32_t n_waiting=0, n_onlink=0, n_atact=0, n_arrived=0;
    for (auto s : hot_.state) {
        switch(s) {
        case AgentState::Waiting:    ++n_waiting;  break;
        case AgentState::OnLink:     ++n_onlink;   break;
        case AgentState::AtActivity: ++n_atact;    break;
        case AgentState::Arrived:    ++n_arrived;  break;
        default: break;
        }
    }
    // Check for suspiciously large travel times (speed=0 edge bug)
    uint32_t slow_edges = 0;
    if (graph_) {
        for (uint32_t e = 0; e < graph_->num_edges(); ++e) {
            float ff = graph_->free_flow_time(e);
            if (ff > 3600.0f) ++slow_edges;   // more than 1 hour on a single edge
        }
    }

    spdlog::info("Event types: Depart={} Enter={} Exit={} Arrive={} Teleported={} DepartRejected={} PretripRerouted={}",
        n_depart_.load(), n_enter_.load(), n_exit_.load(), n_arrive_.load(),
        n_teleported_.load(), n_depart_rejected_.load(), n_pretrip_rerouted_.load());
    spdlog::info("Final states: Waiting={} OnLink={} AtActivity={} Arrived={}",
        n_waiting, n_onlink, n_atact, n_arrived);
    spdlog::info("Queue remaining: {} events | Slow edges (ff>1h): {}",
        eq_.size(), slow_edges);

    // ── Stuck-agent diagnostics ────────────────────────────────────────────────
    // Aggregate over all OnLink agents to understand WHY they haven't arrived.
    const char* class_names[] = {
        "Motorway","MotorwayLink","Trunk","TrunkLink",
        "Primary","PrimaryLink","Secondary","SecondaryLink",
        "Tertiary","TertiaryLink","Residential","LivingStreet",
        "Service","Unclassified","Track","Cycleway","Footway","Path","Steps"
    };
    if (graph_ && n_onlink > 0) {
        // Per road_class: count stuck agents
        std::array<uint32_t, 256> stuck_by_class{};
        // Route completion buckets: 0–25%, 25–50%, 50–75%, 75–99%, 99–100%
        std::array<uint32_t, 5> completion_buckets{};
        // Travel time buckets: <30min, 30–60, 60–120, 120–240, >240 min
        std::array<uint32_t, 5> ttime_buckets{};
        // Top stuck edges (edge_id → count)
        std::unordered_map<EdgeId, uint32_t> edge_counts;
        edge_counts.reserve(4096);
        // reroute_count distribution among currently-stuck agents
        std::array<uint64_t, 32> stuck_by_reroute_count{};

        const uint32_t N = static_cast<uint32_t>(hot_.size());
        for (AgentId a = 0; a < N; ++a) {
            if (hot_.state[a] != AgentState::OnLink) continue;

            EdgeId eid = hot_.current_edge[a];
            if (eid >= graph_->num_edges()) continue;

            uint8_t rc = graph_->edges[eid].road_class;
            stuck_by_class[rc]++;
            edge_counts[eid]++;
            stuck_by_reroute_count[std::min<uint32_t>(hot_.reroute_count[a], stuck_by_reroute_count.size() - 1)]++;

            // Route completion ratio
            auto route = routes_.get_route(a);
            if (!route.empty()) {
                float pct = static_cast<float>(hot_.route_pos[a]) / route.size();
                if      (pct < 0.25f) completion_buckets[0]++;
                else if (pct < 0.50f) completion_buckets[1]++;
                else if (pct < 0.75f) completion_buckets[2]++;
                else if (pct < 0.99f) completion_buckets[3]++;
                else                  completion_buckets[4]++;
            }

            // Time in network (seconds)
            double depart = cold_.plans[a].activities.empty()
                          ? cfg_.start_time
                          : cold_.plans[a].activities[0].start_time;
            double elapsed = until - depart;
            if      (elapsed <  1800) ttime_buckets[0]++;
            else if (elapsed <  3600) ttime_buckets[1]++;
            else if (elapsed <  7200) ttime_buckets[2]++;
            else if (elapsed < 14400) ttime_buckets[3]++;
            else                      ttime_buckets[4]++;
        }

        spdlog::info("── Stuck agents ({}) by current road_class ──", n_onlink);
        for (int rc = 0; rc < 19; ++rc) {
            if (stuck_by_class[rc] > 0)
                spdlog::info("  {:>15s} ({}): {:6d} agents  ({:.1f}%)",
                    class_names[rc], rc, stuck_by_class[rc],
                    100.0f * stuck_by_class[rc] / n_onlink);
        }

        spdlog::info("── Stuck agents by route completion ──");
        const char* cbuckets[] = {"0–25%","25–50%","50–75%","75–99%","≥99%"};
        for (int i = 0; i < 5; ++i)
            if (completion_buckets[i] > 0)
                spdlog::info("  {}: {} agents", cbuckets[i], completion_buckets[i]);

        spdlog::info("── Stuck agents by time in network ──");
        const char* tbuckets[] = {"<30min","30–60min","1–2h","2–4h",">4h"};
        for (int i = 0; i < 5; ++i)
            if (ttime_buckets[i] > 0)
                spdlog::info("  {}: {} agents", tbuckets[i], ttime_buckets[i]);

        // Top 10 most congested stuck-edges
        std::vector<std::pair<uint32_t, EdgeId>> top_edges;
        top_edges.reserve(edge_counts.size());
        for (auto& [eid, cnt] : edge_counts)
            top_edges.push_back({cnt, eid});
        std::partial_sort(top_edges.begin(),
                          top_edges.begin() + std::min<std::size_t>(10, top_edges.size()),
                          top_edges.end(), std::greater<>{});
        spdlog::info("── Top stuck edges (link, road_class, ff_time, n_stuck) ──");
        for (std::size_t i = 0; i < std::min<std::size_t>(10, top_edges.size()); ++i) {
            auto [cnt, eid] = top_edges[i];
            const auto& ed  = graph_->edges[eid];
            int rc = static_cast<int>(ed.road_class);
            float ff = graph_->free_flow_time(eid);
            spdlog::info("  edge {:6d}  class={:2d} ({})  ff={:.0f}s  stuck={}",
                eid, rc, rc < 19 ? class_names[rc] : "?", ff, cnt);
        }

        spdlog::info("── Stuck agents by reroute_count ──");
        for (std::size_t i = 0; i < stuck_by_reroute_count.size(); ++i)
            if (stuck_by_reroute_count[i] > 0)
                spdlog::info("  reroute_count={}: {} agents", i, stuck_by_reroute_count[i]);
    }

    // ── Teleported-agent diagnostics ──────────────────────────────────────────
    // Captured at teleport time (before state is overwritten) across the whole
    // run — shows whether teleported agents exhausted their reroute budget
    // (reroute_count == cfg_.max_reroutes) or were teleported despite having
    // routing headroom left, and which road classes they were last on.
    if (n_teleported_.load() > 0) {
        spdlog::info("── Teleported agents ({}) by reroute_count ──", n_teleported_.load());
        for (std::size_t i = 0; i < teleported_by_reroute_count_.size(); ++i)
            if (teleported_by_reroute_count_[i] > 0)
                spdlog::info("  reroute_count={}: {} agents", i, teleported_by_reroute_count_[i]);

        spdlog::info("── Teleported agents by road_class (at teleport time) ──");
        for (int rc = 0; rc < 19; ++rc)
            if (teleported_by_class_[rc] > 0)
                spdlog::info("  {:>15s} ({}): {:6d} agents",
                    class_names[rc], rc, teleported_by_class_[rc]);
    }

    // ── Network capacity breakdown ────────────────────────────────────────────
    // Reports per-road-class supply (veh/h) and a peak-hour proxy V/C.
    // Peak-hour demand estimate: agents departing in the busiest hour window,
    // divided by the network's total car-class hourly capacity.
    // This is a lower-bound proxy — real V/C is per-link; use link_stats GeoJSON
    // for precise hotspot analysis.
    if (graph_) {
        const char* cls[] = {
            "Motorway","MotorwayLink","Trunk","TrunkLink",
            "Primary","PrimaryLink","Secondary","SecondaryLink",
            "Tertiary","TertiaryLink","Residential","LivingStreet",
            "Service","Unclassified","Track","Cycleway","Footway","Path","Steps"
        };
        std::array<double,  19> cap_sum{};
        std::array<uint32_t,19> edge_cnt{};
        for (uint32_t e = 0; e < graph_->num_edges(); e += 2) {
            const auto& ed = graph_->edges[e];
            if (ed.road_class < 19) {
                cap_sum[ed.road_class]  += ed.capacity;
                edge_cnt[ed.road_class]++;
            }
        }
        // Car-accessible classes only (0 = Motorway … 13 = Unclassified)
        double car_cap_veh_h = 0.0;
        for (int rc = 0; rc <= 13; ++rc) car_cap_veh_h += cap_sum[rc];

        // Peak-hour proxy: busiest 1-hour window of departures.
        // Bin agents by departure hour, find max.
        std::array<uint32_t, 24> hour_bins{};
        for (AgentId a = 0; a < static_cast<AgentId>(cold_.plans.size()); ++a) {
            if (cold_.plans[a].activities.empty()) continue;
            SimTime dep = cold_.plans[a].activities[0].start_time;
            int h = std::clamp(static_cast<int>(dep / 3600.0f), 0, 23);
            hour_bins[h]++;
        }
        uint32_t peak_agents = *std::max_element(hour_bins.begin(), hour_bins.end());
        int      peak_hour   = static_cast<int>(
            std::max_element(hour_bins.begin(), hour_bins.end()) - hour_bins.begin());

        spdlog::info("── Network capacity breakdown (directional edges) ──");
        spdlog::info("  Car-class network capacity: {:.0f} veh/h total", car_cap_veh_h);
        spdlog::info("  Peak departure hour: {:02d}:00  ({} agents departing)", peak_hour, peak_agents);
        if (car_cap_veh_h > 0.0)
            spdlog::info("  Peak-hour proxy V/C: {:.4f}  "
                         "(peak departures / total car-class cap — lower bound, real V/C is per-link)",
                         static_cast<double>(peak_agents) / car_cap_veh_h);
        spdlog::info("  Road class (directional edges, cap veh/h, share of car-class cap):");
        for (int rc = 0; rc <= 14; ++rc) {
            if (edge_cnt[rc] == 0) continue;
            spdlog::info("    {:>15s}: {:5d} edges  {:8.0f} veh/h  ({:.1f}%)",
                         cls[rc], edge_cnt[rc], cap_sum[rc],
                         car_cap_veh_h > 0 ? 100.0 * cap_sum[rc] / car_cap_veh_h : 0.0);
        }
    }

    spdlog::info("Simulation complete in {:.1f}s.",
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count());
}

void Simulation::process_events_until(SimTime wend) {
    // Always sequential. Parallel dispatch requires per-agent locks on
    // EventQueue, RouteStore and AgentHotStore — planned Phase 3. Enabling TBB
    // parallel_for here without those locks causes data races on shared
    // vectors → segfault.
    std::vector<Event> batch;
    if (cfg_.event_step_s <= 0.0f) {
        eq_.drain_until(wend, batch);
        for (const auto& ev : batch) handle_event(ev);
        return;
    }

    // Handling an event usually schedules the agent's next one (exit of the
    // edge just entered, 1 s retry of a blocked exit/departure). Those must
    // run at their own time, not a whole sync window later — so each slice is
    // drained repeatedly until nothing earlier than its end is left. Every
    // follow-up is strictly later than its cause or advances the agent along
    // a finite route, so the inner loop terminates.
    SimTime slice_end = current_time_.load();
    while (slice_end < wend) {
        slice_end = std::min<SimTime>(slice_end + cfg_.event_step_s, wend);
        for (;;) {
            batch.clear();
            eq_.drain_until(slice_end, batch);
            if (batch.empty()) break;
            for (const auto& ev : batch) handle_event(ev);
        }
    }
}

void Simulation::step() {
    if (eq_.empty() || current_time_.load() >= cfg_.end_time) return;
    SimTime wend = current_time_.load() + cfg_.sync_window_s;
    process_events_until(wend);
    flush_traffic_state();
    current_time_.store(wend);
}

std::size_t Simulation::active_agents() const noexcept {
    std::size_t n = 0;
    for (const auto& s : hot_.state)
        if (s == AgentState::OnLink || s == AgentState::Waiting) ++n;
    return n;
}

const Graph& Simulation::graph() const {
    if (!graph_) throw std::runtime_error("Graph not set");
    return *graph_;
}

const ITrafficModel& Simulation::traffic() const {
    if (!traffic_) throw std::runtime_error("Traffic model not set");
    return *traffic_;
}


void Simulation::enable_td_recording(float bin_s) {
    if (!graph_) throw std::runtime_error("enable_td_recording: graph not set");
    td_rec_ = std::make_unique<TdCostRecorder>(
        static_cast<uint32_t>(graph_->num_edges()), cfg_.start_time, cfg_.end_time, bin_s);
}

void Simulation::write_td_costs(const std::filesystem::path& p) const {
    if (!td_rec_) throw std::runtime_error("write_td_costs: recording not enabled");
    // Copy so the censored observations do not leak into a later run_until().
    TdCostRecorder rec = *td_rec_;
    const SimTime now = current_time_.load();
    std::size_t censored = 0;
    for (AgentId a = 0; a < static_cast<AgentId>(hot_.size()); ++a) {
        if (hot_.state[a] != AgentState::OnLink || hot_.mode[a] != AgentMode::Car) continue;
        rec.record(hot_.current_edge[a], hot_.enter_time[a], now);
        ++censored;
    }
    rec.table().save(p);
    spdlog::info("Time-dependent link costs written to {} ({} cars censored at t={:.0f}s)",
                 p.string(), censored, now);
}

} // namespace nomad
