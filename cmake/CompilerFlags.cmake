# Compiler flags for nomad — applied globally
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
)

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-O3 -march=native)

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
