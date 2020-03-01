cmake_minimum_required(VERSION 3.15)

# Define application settings.
if (NOT DEFINED APP)
    message(FATAL_ERROR "'APP' is not defined!")
endif ()

include(test/${APP}/${APP}.cmake)

include(cmake/conan.cmake)
include(cmake/coverage.cmake)
include(cmake/doxygen.cmake)
include(cmake/osal.cmake)
include(cmake/platform.cmake)

project(hal VERSION 1.0 LANGUAGES ASM C CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project-wide compilation options.
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Generate coverage report.
if (COVERAGE)
    add_lcov_coverage(${platform_SOURCE_DIR}/app/* ${osal_SOURCE_DIR}/lib/* ${CMAKE_CURRENT_SOURCE_DIR}/test/*)
endif ()

add_subdirectory(${platform_SOURCE_DIR}/app ${platform_BINARY_DIR})
add_subdirectory(${osal_SOURCE_DIR}/lib ${osal_BINARY_DIR})
add_subdirectory(lib)
add_subdirectory(test)

# Generate documentation.
doxygen_addTarget(${CMAKE_CURRENT_SOURCE_DIR}/lib README.md)
