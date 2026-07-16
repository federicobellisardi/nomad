#include <nomad/core/simulation.hpp>
#include <nomad/traffic/traffic_model.hpp>
#include <nomad/routing/router.hpp>
#include <nomad/routing/astar_router.hpp>
#include <random>
#include <chrono>
#include <nomad/demand/demand_model.hpp>
#include <nomad/output/writer.hpp>

#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <tbb/task_arena.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <spdlog/spdlog.h>

namespace nomad {

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
    spdlog::info("Simulation: {} agents injected", n);

    if (!router_ || n == 0) return;

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
    std::unique_ptr<AStarRouter> stoch_router;
    if (sigma > 0.0f) {
        stoch_router = std::make_unique<AStarRouter>(
            *graph_, nullptr, AStarRouter::Config{false, 33.3f});
        spdlog::info("Per-edge stochastic pre-routing enabled (sigma={:.2f})", sigma);
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
                    RoutingRequest req{
                        plan.activities[0].location,
                        plan.activities[1].location,
                        plan.activities[0].start_time,
                        plan.preferred_mode
                    };

                    if (stoch_router) {
                        uint32_t seed = 0xCA7F00Du ^ (static_cast<uint32_t>(a) * 2654435761u);
                        Route r = stoch_router->route_stochastic(
                            req.origin, req.destination, req.mode, seed, sigma);

                        // Reject extreme detours: > 2× crow-fly AND > 5 km.
                        // Falls back to CH so bad hashes never lose an agent.
                        float haversine_m = graph_->haversine(req.origin, req.destination);
                        bool too_long = r.estimated_dist_m > 2.0f * haversine_m
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
            float tt = 0;
            for (EdgeId eid : edges) {
                if (eid < graph_->num_edges())
                    tt += graph_->free_flow_time(eid);
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
    case EventType::AgentEnterLink:      ++n_enter_;   handle_enter_link(e);       break;
    case EventType::AgentExitLink:       ++n_exit_;    handle_exit_link(e);        break;
    case EventType::AgentArriveActivity: ++n_arrive_;  handle_arrive_activity(e);  break;
    case EventType::AgentReroute:        handle_reroute(e);          break;
    case EventType::SignalPhaseChange:   handle_signal_change(e);    break;
    default: break;
    }
    fire_hooks(e.type, e);
}

void Simulation::handle_depart(const Event& e) {
    AgentId a = e.agent;
    if (a >= hot_.size()) return;
    hot_.state[a] = AgentState::OnLink;

    // Try pre-computed route first (set by inject_demand's pre-routing phase)
    auto pre_edges = routes_.get_route(a);
    if (!pre_edges.empty()) {
        hot_.route_pos[a]    = 0;
        hot_.current_edge[a] = pre_edges[0];
        eq_.push({e.time, a, pre_edges[0], EventType::AgentEnterLink, {}});
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

    routes_.push_route(a, route.edges);
    hot_.route_pos[a]   = 0;
    hot_.current_edge[a]= route.edges[0];

    Event enter{e.time, a, route.edges[0], EventType::AgentEnterLink, {}};
    eq_.push(enter);
}

void Simulation::handle_enter_link(const Event& e) {
    AgentId a   = e.agent;
    EdgeId  eid = static_cast<EdgeId>(e.payload);
    if (a >= hot_.size()) return;
    if (hot_.state[a] == AgentState::Arrived) return;  // already teleported
    if (!graph_ || eid >= graph_->num_edges()) {
        hot_.state[a] = AgentState::Arrived;
        return;
    }

    hot_.current_edge[a] = eid;
    hot_.enter_time[a]   = e.time;
    hot_.state[a]        = AgentState::OnLink;

    SimTime exit_time = e.time + graph_->free_flow_time(eid);
    if (traffic_) exit_time = traffic_->on_enter(eid, a, e.time);

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

    bool can_exit = true;
    if (traffic_) can_exit = traffic_->on_exit(eid, a, e.time);

    if (!can_exit) {
        eq_.push({e.time + 1.0, a, eid, EventType::AgentExitLink, {}});
        return;
    }

    uint16_t pos = ++hot_.route_pos[a];
    auto route = routes_.get_route(a);

    if (pos >= route.size()) {
        hot_.state[a] = AgentState::AtActivity;
        eq_.push({e.time, a, eid, EventType::AgentArriveActivity, {}});
        return;
    }

    EdgeId next = route[pos];
    eq_.push({e.time, a, next, EventType::AgentEnterLink, {}});
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
        routes_.replace_suffix(a, hot_.route_pos[a], r.edges);
        // Keep freeflow_route_s in sync with the rerouted path so the stuck
        // threshold (ratio × ff) is never exceeded by BPR-congested agents on
        // a longer but legitimate detour.
        auto full = routes_.get_route(a);
        float ff = 0;
        for (EdgeId eid : full)
            if (eid < graph_->num_edges())
                ff += graph_->free_flow_time(eid);
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
        if (eid < graph_->num_edges() && traffic_)
            traffic_->force_remove(eid, a);
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

    // 1. Identify congested edges (tt > (1+thresh) × ff).
    std::vector<EdgeId> congested_vec;
    for (uint32_t eid = 0; eid < E; ++eid) {
        float occ = states[eid].occupancy.load(std::memory_order_relaxed);
        if (occ == 0.0f) continue;
        float tt = states[eid].travel_time_s.load(std::memory_order_relaxed);
        float ff = graph_->free_flow_time(eid);
        if (ff > 0.0f && tt / ff > 1.0f + cfg_.reroute_thresh)
            congested_vec.push_back(eid);
    }
    std::unordered_set<EdgeId> congested_set(congested_vec.begin(), congested_vec.end());

    if (congested_vec.empty()) return;

    // Update CH cost table so future departures see updated costs.
    if (router_) router_->update_costs(congested_vec);

    IRouter* r_ptr = reroute_router_ ? static_cast<IRouter*>(reroute_router_.get())
                                     : router_.get();
    if (!r_ptr) return;

    // 2. Collect agents whose upcoming route crosses a congested link.
    //    Skip agents that have already been rerouted max_reroutes times —
    //    repeated rerouting of the same agent causes route oscillation (routing loop).
    const uint32_t N = static_cast<uint32_t>(hot_.size());
    std::vector<AgentId> to_reroute;
    to_reroute.reserve(std::min<uint32_t>(N, 65536u));
    for (AgentId a = 0; a < N; ++a) {
        if (hot_.state[a] != AgentState::OnLink) continue;
        if (cfg_.max_reroutes > 0 && hot_.reroute_count[a] >= cfg_.max_reroutes) continue;
        auto route    = routes_.get_route(a);
        uint16_t pos  = hot_.route_pos[a];
        constexpr int kLookAhead = 5;
        for (uint32_t k = 1; k <= kLookAhead && pos + k < route.size(); ++k) {
            if (congested_set.count(route[pos + k])) {
                to_reroute.push_back(a);
                break;
            }
        }
    }

    if (to_reroute.empty()) return;

    // 3. Compute new routes in parallel.
    //    A* (reroute_router_) is thread-safe: each thread uses its own workspace.
    //    hot_ and cold_ are read-only here (no events processing concurrently).
    std::vector<Route> new_routes(to_reroute.size());
    impl_->arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, to_reroute.size(), 32),
            [&](const tbb::blocked_range<std::size_t>& rng) {
                for (std::size_t i = rng.begin(); i < rng.end(); ++i) {
                    AgentId a = to_reroute[i];
                    if (cold_.plans[a].activities.size() < 2) continue;
                    EdgeId cur = hot_.current_edge[a];
                    if (cur >= graph_->num_edges()) continue;
                    NodeId from = graph_->edges[cur].target;
                    NodeId dest = cold_.plans[a].activities.back().location;
                    RoutingRequest req{from, dest, now, hot_.mode[a]};
                    new_routes[i] = r_ptr->route(req);
                }
            });
    });

    // 4. Apply results sequentially (RouteStore::replace_suffix writes to shared
    //    flat_data — not thread-safe for concurrent agents).
    std::size_t n_applied = 0;
    for (std::size_t i = 0; i < to_reroute.size(); ++i) {
        AgentId a = to_reroute[i];
        if (new_routes[i].is_valid && !new_routes[i].edges.empty()) {
            routes_.replace_suffix(a, hot_.route_pos[a], new_routes[i].edges);
            // Keep freeflow_route_s in sync with the rerouted path.
            auto full = routes_.get_route(a);
            float ff = 0;
            for (EdgeId eid : full)
                if (eid < graph_->num_edges())
                    ff += graph_->free_flow_time(eid);
            hot_.freeflow_route_s[a] = ff;
            ++hot_.reroute_count[a];
            ++n_applied;
        }
    }

    spdlog::debug("schedule_reroutes t={:.0f}: {} congested links, {}/{} agents rerouted",
                  now, congested_vec.size(), n_applied, to_reroute.size());
}

void Simulation::run() { run_until(cfg_.end_time); }

void Simulation::run_until(SimTime until) {
    if (!graph_) throw std::runtime_error("Simulation::run_until: graph not set");

    // Build the traffic-aware A* router used for en-route rerouting.
    // Created here so it sees the already-set traffic_ model.
    if (!reroute_router_ && graph_ && traffic_) {
        AStarRouter::Config acfg;
        acfg.use_traffic_costs = true;
        reroute_router_ = std::make_unique<AStarRouter>(*graph_, traffic_.get(), acfg);
        spdlog::info("Reroute router: A* with traffic costs");
    }

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

        std::vector<Event> batch;
        eq_.drain_until(wend, batch);

        // Phase 1: always sequential. Parallel dispatch requires per-agent
        // locks on EventQueue, RouteStore and AgentHotStore — planned Phase 3.
        // Enabling TBB parallel_for here without those locks causes data races
        // on shared vectors → segfault.
        for (const auto& ev : batch) handle_event(ev);

        flush_traffic_state();
        schedule_reroutes();
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

    spdlog::info("Event types: Depart={} Enter={} Exit={} Arrive={} Teleported={}",
        n_depart_.load(), n_enter_.load(), n_exit_.load(), n_arrive_.load(),
        n_teleported_.load());
    spdlog::info("Final states: Waiting={} OnLink={} AtActivity={} Arrived={}",
        n_waiting, n_onlink, n_atact, n_arrived);
    spdlog::info("Queue remaining: {} events | Slow edges (ff>1h): {}",
        eq_.size(), slow_edges);

    // ── Stuck-agent diagnostics ────────────────────────────────────────────────
    // Aggregate over all OnLink agents to understand WHY they haven't arrived.
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

        const uint32_t N = static_cast<uint32_t>(hot_.size());
        for (AgentId a = 0; a < N; ++a) {
            if (hot_.state[a] != AgentState::OnLink) continue;

            EdgeId eid = hot_.current_edge[a];
            if (eid >= graph_->num_edges()) continue;

            uint8_t rc = graph_->edges[eid].road_class;
            stuck_by_class[rc]++;
            edge_counts[eid]++;

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
        const char* class_names[] = {
            "Motorway","MotorwayLink","Trunk","TrunkLink",
            "Primary","PrimaryLink","Secondary","SecondaryLink",
            "Tertiary","TertiaryLink","Residential","LivingStreet",
            "Service","Unclassified","Track","Cycleway","Footway","Path","Steps"
        };
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

void Simulation::step() {
    if (eq_.empty() || current_time_.load() >= cfg_.end_time) return;
    SimTime wend = current_time_.load() + cfg_.sync_window_s;
    std::vector<Event> batch;
    eq_.drain_until(wend, batch);
    for (const auto& ev : batch) handle_event(ev);
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

} // namespace nomad
