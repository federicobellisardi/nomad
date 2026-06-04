#pragma once

#include <nomad/core/agent.hpp>
#include <nomad/core/event.hpp>
#include <nomad/core/graph.hpp>
#include <nomad/core/types.hpp>

#include <string_view>

namespace nomad {

// ── IDemandModel ──────────────────────────────────────────────────────────────
// All demand models implement this interface.
// generate() is called once before the simulation starts.
// It is responsible for:
//   1. Creating agent cold-store entries (activity plans)
//   2. Inserting AgentDepart events into the event queue
//
// The simulation engine sizes its hot store based on the number of agents
// registered here. generate() must therefore be called BEFORE hot_.resize().
class IDemandModel {
public:
    virtual ~IDemandModel() = default;

    // Populate the stores and insert departure events.
    // agents_out: number of agents created (used to size hot stores)
    virtual std::size_t generate(const Graph&     graph,
                                  EventQueue&      eq,
                                  AgentColdStore&  cold,
                                  RouteStore&      routes) = 0;

    virtual std::string_view model_name() const = 0;
};

} // namespace nomad
