cmake_minimum_required(VERSION 3.24)

# Define application settings.
if (NOT DEFINED APP)
    message(FATAL_ERROR "'APP' is not defined!")
endif ()

include(test/${APP}/settings.cmake)

include(cmake/conan.cmake)
include(cmake/coverage.cmake)
include(cmake/doxygen.cmake)
include(cmake/osal.cmake)
include(cmake/platform.cmake)
include(cmake/sanitizers.cmake)
include(cmake/utils.cmake)

project(hal VERSION 1.0.0 LANGUAGES ASM C CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

if (USE_CONAN)
    conan_init_wrapper()
    conan_init_profile()
    conan_cmake_install(PATH_OR_REFERENCE ${CMAKE_SOURCE_DIR} BUILD missing PROFILE_BUILD default PROFILE_HOST conan-profile)
    conan_load_buildinfo()
    conan_basic_setup()
endif ()

# Project-wide compilation options.
add_compile_options(-Wall -Wextra -Wpedantic -Werror $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>)

if (USE_ASAN OR USE_LSAN OR USE_UBSAN)
    enable_sanitizers()
endif ()

if (COVERAGE)
    add_lcov_coverage(IGNORE ${CMAKE_CURRENT_SOURCE_DIR}/test/* */external/*)
endif ()

add_subdirectory(${platform_SOURCE_DIR}/lib ${platform_BINARY_DIR})
add_subdirectory(${osal_SOURCE_DIR}/lib ${osal_BINARY_DIR})
add_subdirectory(${utils_SOURCE_DIR}/lib ${utils_BINARY_DIR})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(lib)
add_subdirectory(test)

doxygen_addTarget(${CMAKE_CURRENT_SOURCE_DIR}/lib README.md)
