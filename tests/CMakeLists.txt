add_executable(nomad_tests
    unit/test_graph.cpp
    unit/test_routing.cpp
    unit/test_traffic.cpp
    unit/test_event_queue.cpp
    unit/test_agent_store.cpp
    unit/test_od_matrix.cpp
    integration/test_simulation_synthetic.cpp
)

target_link_libraries(nomad_tests PRIVATE
    nomad_core
    Catch2::Catch2WithMain
)

catch_discover_tests(nomad_tests)
