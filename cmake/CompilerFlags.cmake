# Compiler flags for nomad — applied globally
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
)

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    # -march=native bakes in whatever ISA extensions the BUILDING machine's
    # CPU happens to have (e.g. AVX-512 on this dev node's AMD EPYC 9474F).
    # On a heterogeneous SLURM cluster, a node whose CPU lacks one of those
    # extensions raises SIGILL the instant it hits that instruction -- not a
    # crash in nomad's own logic. Confirmed directly: a 55-job city/date
    # sweep run from this build had 51/55 jobs die with SIGILL, all on nodes
    # other than the one this binary happened to be built on (slurm24) or
    # its apparent hardware twins (slurm21/25); the 4 survivors ran only on
    # those. x86-64-v2 (SSE3/4.1/4.2, POPCNT) has been part of essentially
    # every x86-64 CPU shipped since ~2009 -- safe across any node this
    # cluster is likely to have, at some cost to AVX2/FMA-level throughput.
    add_compile_options(-O3 -march=x86-64-v2)

    if(NOMAD_USE_LTO)
        include(CheckIPOSupported)
        check_ipo_supported(RESULT ipo_ok)
        if(ipo_ok)
            set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
        endif()
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -fno-omit-frame-pointer)
endif()
