include(FetchContent)

# ── libosmium (OSM PBF parsing — header-only) ─────────────────────────────────
# libosmium has its own CMakeLists.txt that tries find_package(Osmium) and fails.
# We use FetchContent_Populate (download-only, no add_subdirectory) and set
# include paths manually. This is the correct pattern for header-only libs
# whose own CMakeLists.txt are not suitable for subdirectory inclusion.
FetchContent_Declare(
    libosmium
    GIT_REPOSITORY https://github.com/osmcode/libosmium.git
    GIT_TAG        v2.20.0
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(
    protozero
    GIT_REPOSITORY https://github.com/mapbox/protozero.git
    GIT_TAG        v1.7.1
    GIT_SHALLOW    TRUE
)

# Download-only (skip add_subdirectory) — both are header-only
FetchContent_GetProperties(libosmium)
if(NOT libosmium_POPULATED)
    FetchContent_Populate(libosmium)
endif()

FetchContent_GetProperties(protozero)
if(NOT protozero_POPULATED)
    FetchContent_Populate(protozero)
endif()

# Create a clean interface target for libosmium
add_library(osmium INTERFACE)
target_include_directories(osmium INTERFACE
    ${libosmium_SOURCE_DIR}/include
    ${protozero_SOURCE_DIR}/include
)
find_package(ZLIB REQUIRED)
target_link_libraries(osmium INTERFACE ZLIB::ZLIB)
target_compile_definitions(osmium INTERFACE
    OSMIUM_WITH_SPARSEHASH=0
)
# Optional compression support
find_package(BZip2 QUIET)
if(BZip2_FOUND OR BZIP2_FOUND)
    target_link_libraries(osmium INTERFACE BZip2::BZip2)
    target_compile_definitions(osmium INTERFACE OSMIUM_WITH_BZIP2=1)
endif()
find_package(LibLZMA QUIET)
if(LibLZMA_FOUND OR LIBLZMA_FOUND)
    target_link_libraries(osmium INTERFACE LibLZMA::LibLZMA)
    target_compile_definitions(osmium INTERFACE OSMIUM_WITH_LZMA=1)
endif()

# ── oneTBB (task parallelism) ─────────────────────────────────────────────────
find_package(TBB QUIET CONFIG)
if(NOT TBB_FOUND)
    find_package(TBB QUIET)
endif()
if(NOT TBB_FOUND)
    message(STATUS "TBB not found via find_package, fetching from source...")
    FetchContent_Declare(
        onetbb
        GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
        GIT_TAG        v2021.12.0
        GIT_SHALLOW    TRUE
    )
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(onetbb)
else()
    message(STATUS "Found TBB: ${TBB_VERSION}")
endif()

# ── Apache Arrow + Parquet ────────────────────────────────────────────────────
# Prefer system/conda installation; skip if not found (Parquet output is optional).
find_package(Arrow QUIET CONFIG)
find_package(Parquet QUIET CONFIG)
if(Arrow_FOUND AND Parquet_FOUND)
    message(STATUS "Found Arrow: ${ARROW_VERSION}, Parquet: ${Parquet_VERSION}")
else()
    message(STATUS "Apache Arrow/Parquet not found — Parquet output will be disabled.")
    message(STATUS "Install with: conda install -c conda-forge arrow-cpp")
    message(STATUS "Or: sudo apt install libarrow-dev libparquet-dev")
    if(NOT TARGET Arrow::arrow_static)
        add_library(Arrow::arrow_static INTERFACE IMPORTED)
    endif()
    if(NOT TARGET Parquet::parquet_static)
        add_library(Parquet::parquet_static INTERFACE IMPORTED)
    endif()
endif()

# ── nlohmann/json ─────────────────────────────────────────────────────────────
find_package(nlohmann_json QUIET CONFIG)
if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
    )
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# ── spdlog ────────────────────────────────────────────────────────────────────
find_package(spdlog QUIET CONFIG)
if(NOT spdlog_FOUND)
    FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG        v1.13.0
        GIT_SHALLOW    TRUE
    )
    set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    FetchContent_MakeAvailable(spdlog)
endif()

# ── yaml-cpp ──────────────────────────────────────────────────────────────────
find_package(yaml-cpp QUIET CONFIG)
if(NOT yaml-cpp_FOUND)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG        0.8.0
        GIT_SHALLOW    TRUE
    )
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL    OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# ── Boost (headers only for Geometry, optional) ───────────────────────────────
find_package(Boost 1.74 QUIET COMPONENTS headers)
if(NOT Boost_FOUND)
    message(STATUS "Boost not found. Spatial zone→node join will be unavailable.")
    if(NOT TARGET Boost::headers)
        add_library(Boost::headers INTERFACE IMPORTED)
    endif()
endif()

# ── Catch2 (unit tests) ───────────────────────────────────────────────────────
if(NOMAD_BUILD_TESTS)
    find_package(Catch2 3 QUIET CONFIG)
    if(NOT Catch2_FOUND)
        FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        v3.5.2
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(Catch2)
        list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
    endif()
    include(CTest)
    include(Catch)
endif()

# ── Google Benchmark ──────────────────────────────────────────────────────────
if(NOMAD_BUILD_BENCHMARKS)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG        v1.8.3
        GIT_SHALLOW    TRUE
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
endif()

# ── pybind11 ──────────────────────────────────────────────────────────────────
if(NOMAD_BUILD_PYTHON)
    find_package(pybind11 QUIET CONFIG)
    if(NOT pybind11_FOUND)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.11.1
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(pybind11)
    endif()
endif()
