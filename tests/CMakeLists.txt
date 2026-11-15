add_executable(nomad_tests
    unit/test_graph.cpp
    unit/test_routing.cpp
    unit/test_traffic.cpp
    unit/test_event_queue.cpp
    unit/test_agent_store.cpp
    unit/test_od_matrix.cpp
    unit/test_network_cleaner.cpp
    unit/test_intersection.cpp
    unit/test_walk_access.cpp
    unit/test_movement_storage.cpp
    unit/test_stuck_time.cpp
    unit/test_td_costs.cpp
    unit/test_implied_oneway.cpp
    unit/test_osm_lanes.cpp
    unit/test_priority.cpp
    unit/test_osm_derating.cpp
    unit/test_qnode.cpp
    unit/test_replanning.cpp
    integration/test_simulation_synthetic.cpp
)

target_link_libraries(nomad_tests PRIVATE
    nomad_core
    Catch2::Catch2WithMain
)

catch_discover_tests(nomad_tests)
