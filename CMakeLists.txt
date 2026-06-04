cmake_minimum_required(VERSION 3.20)
project(nomad VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Options ──────────────────────────────────────────────────────────────────
option(NOMAD_BUILD_PYTHON     "Build pybind11 Python module"      ON)
option(NOMAD_BUILD_TESTS      "Build unit and integration tests"   ON)
option(NOMAD_BUILD_BENCHMARKS "Build Google Benchmark suite"       OFF)
option(NOMAD_USE_ASAN         "Enable AddressSanitizer"            OFF)
option(NOMAD_USE_LTO          "Enable Link-Time Optimisation"      OFF)

# ── CMake modules ─────────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake/CompilerFlags.cmake)
include(cmake/Dependencies.cmake)

# ── Core library ─────────────────────────────────────────────────────────────
add_library(nomad_core STATIC
    src/core/graph.cpp
    src/core/agent.cpp
    src/core/event.cpp
    src/core/simulation.cpp
    src/network/intersection.cpp
    src/network/osm_loader.cpp
    src/network/network_cleaner.cpp
    src/network/multilayer_graph.cpp
    src/traffic/queue_model.cpp
    src/traffic/ltm_model.cpp
    src/routing/astar_router.cpp
    src/routing/ch_router.cpp
    src/routing/route_cache.cpp
    src/demand/od_matrix.cpp
    src/demand/synthetic.cpp
    src/demand/activity_plan.cpp
    src/demand/mode_choice.cpp
    src/transit/gtfs_loader.cpp
    src/transit/transit_model.cpp
    src/output/parquet_writer.cpp
    src/output/geojson_writer.cpp
    src/analytics/aggregator.cpp
    src/config/scenario_config.cpp
)

target_include_directories(nomad_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(nomad_core PUBLIC
    osmium
    TBB::tbb
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    Boost::headers
)

# Apache Arrow / Parquet are optional — enabled if found
if(Arrow_FOUND AND Parquet_FOUND)
    target_link_libraries(nomad_core PUBLIC
        Arrow::arrow_shared
        Parquet::parquet_shared
    )
    target_compile_definitions(nomad_core PUBLIC NOMAD_HAS_ARROW=1)
    message(STATUS "Parquet output: ENABLED")
else()
    message(STATUS "Parquet output: DISABLED (install arrow-cpp to enable)")
endif()

if(NOMAD_USE_ASAN)
    target_compile_options(nomad_core PUBLIC -fsanitize=address,undefined)
    target_link_options   (nomad_core PUBLIC -fsanitize=address,undefined)
endif()

# ── Python extension (_nomad_core) ────────────────────────────────────────────
if(NOMAD_BUILD_PYTHON)
    find_package(Python COMPONENTS Interpreter Development REQUIRED)
    # pybind11 already fetched/found in Dependencies.cmake
    pybind11_add_module(_nomad_core bindings/pynomad.cpp)
    target_link_libraries(_nomad_core PRIVATE nomad_core)
    set_target_properties(_nomad_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python/nomad"
    )
endif()

# ── CLI executable ────────────────────────────────────────────────────────────
add_executable(nomad_cli tools/nomad-cli/main.cpp)
target_link_libraries(nomad_cli PRIVATE nomad_core)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(NOMAD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ── Benchmarks ────────────────────────────────────────────────────────────────
if(NOMAD_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
