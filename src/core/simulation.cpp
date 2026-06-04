#include <nomad/core/simulation.hpp>
#include <nomad/traffic/traffic_model.hpp>
#include <nomad/routing/router.hpp>
#include <chrono>
#include <nomad/demand/demand_model.hpp>
#include <nomad/output/writer.hpp>

#include <stdexcept>
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
    // This moves O(N_agents × A*_cost) out of the event loop.
    // handle_depart() will find the pre-computed route in routes_ and skip routing.
    spdlog::info("Pre-routing {} agents (this is the slow step)...", n);
    std::size_t routed = 0, failed = 0;
    auto t0 = std::chrono::steady_clock::now();

    for (AgentId a = 0; a < static_cast<AgentId>(n); ++a) {
        const auto& plan = cold_.plans[a];
        if (plan.activities.size() < 2) { ++failed; continue; }
        RoutingRequest req{
            plan.activities[0].location,
            plan.activities[1].location,
            plan.activities[0].start_time,
            plan.preferred_mode
        };
        Route r = router_->route(req);
        if (r.is_valid && !r.edges.empty()) {
            routes_.push_route(a, r.edges);
            ++routed;
        } else {
            ++failed;
        }
        if ((a + 1) % 5000 == 0) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            double rate = (a + 1) / elapsed;
            double eta  = (n - a - 1) / rate;
            spdlog::info("  Pre-routed {}/{} agents ({} failed) — {:.0f} routes/s — ETA {:.0f}s",
                a + 1, n, failed, rate, eta);
        }
    }
    auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    spdlog::info("Pre-routing done: {}/{} agents routed in {:.1f}s ({:.0f} routes/s)",
        routed, n, elapsed, routed / std::max(elapsed, 0.001));

    // ── Diagnostics: trip duration distribution ────────────────────────────────
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
            SimTime t_dep = cold_.plans[a].activities[0].start_time;
            if (t_dep + tt > cfg_.end_time) ++beyond_window;
            else ++will_finish;
            max_trip_s = std::max(max_trip_s, tt);
            sum_trip_s += tt;
            ++n_sampled;
        }
        float avg = n_sampled > 0 ? sum_trip_s / n_sampled : 0;
        spdlog::info("Trip durations: avg={:.0f}s max={:.0f}s | "
                     "will_finish={} beyond_sim_window={}",
                     avg, max_trip_s, will_finish, beyond_window);
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
    if (a >= hot_.size() || !router_ || !graph_) return;

    const auto& plan = cold_.plans[a];
    if (plan.activities.size() < 2) return;

    NodeId dest = plan.activities.back().location;
    EdgeId cur  = hot_.current_edge[a];
    if (cur >= graph_->num_edges()) return;
    NodeId from = graph_->edges[cur].target;

    RoutingRequest req{from, dest, e.time, hot_.mode[a]};
    Route r = router_->route(req);
    if (r.is_valid && !r.edges.empty())
        routes_.replace_suffix(a, hot_.route_pos[a], r.edges);
}

void Simulation::handle_signal_change(const Event&) {}

void Simulation::flush_traffic_state() {
    SimTime t = current_time_.load();
    if (traffic_) traffic_->update(t);
    for (auto& w : writers_)
        if (graph_ && traffic_) w->on_snapshot(t, hot_, *graph_, *traffic_);
}

void Simulation::schedule_reroutes() {
    if (!traffic_ || !graph_ || !router_) return;
    const auto states = traffic_->link_states();
    std::vector<EdgeId> congested;
    for (uint32_t eid = 0; eid < graph_->num_edges(); ++eid) {
        float tt = states[eid].travel_time_s.load();
        float ff = graph_->free_flow_time(eid);
        if (ff > 0.0f && tt / ff > 1.0f + cfg_.reroute_thresh)
            congested.push_back(eid);
    }
    if (!congested.empty()) router_->update_costs(congested);
}

void Simulation::run() { run_until(cfg_.end_time); }

void Simulation::run_until(SimTime until) {
    if (!graph_) throw std::runtime_error("Simulation::run_until: graph not set");

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

    spdlog::info("Event types: Depart={} Enter={} Exit={} Arrive={}",
        n_depart_.load(), n_enter_.load(), n_exit_.load(), n_arrive_.load());
    spdlog::info("Final states: Waiting={} OnLink={} AtActivity={} Arrived={}",
        n_waiting, n_onlink, n_atact, n_arrived);
    spdlog::info("Queue remaining: {} events | Slow edges (ff>1h): {}",
        eq_.size(), slow_edges);
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
