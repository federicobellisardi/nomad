#include <nomad/core/simulation.hpp>
#include <nomad/traffic/traffic_model.hpp>
#include <nomad/routing/router.hpp>
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
}

void Simulation::handle_event(const Event& e) {
    ++events_processed_;
    switch (e.type) {
    case EventType::AgentDepart:         handle_depart(e);           break;
    case EventType::AgentEnterLink:      handle_enter_link(e);       break;
    case EventType::AgentExitLink:       handle_exit_link(e);        break;
    case EventType::AgentArriveActivity: handle_arrive_activity(e);  break;
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

    const auto& plan = cold_.plans[a];
    if (plan.activities.size() < 2) {
        hot_.state[a] = AgentState::Arrived;
        return;
    }
    NodeId origin = plan.activities[0].location;
    NodeId dest   = plan.activities[1].location;

    if (!router_) { hot_.state[a] = AgentState::Arrived; return; }

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

    hot_.current_edge[a] = eid;
    hot_.enter_time[a]   = e.time;
    hot_.state[a]        = AgentState::OnLink;

    SimTime exit_time = e.time;
    if (traffic_) {
        exit_time = traffic_->on_enter(eid, a, e.time);
    } else if (graph_ && eid < graph_->num_edges()) {
        exit_time = e.time + graph_->free_flow_time(eid);
    }

    hot_.scheduled_exit[a] = exit_time;
    eq_.push({exit_time, a, eid, EventType::AgentExitLink, {}});
}

void Simulation::handle_exit_link(const Event& e) {
    AgentId a   = e.agent;
    EdgeId  eid = static_cast<EdgeId>(e.payload);
    if (a >= hot_.size()) return;

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

    while (!eq_.empty() && current_time_.load() < until) {
        SimTime wend = current_time_.load() + cfg_.sync_window_s;

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
    spdlog::info("Done. Events processed: {}, final active agents: {}",
                  events_processed_.load(), active_agents());
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
