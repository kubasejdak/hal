cmake_minimum_required(VERSION 3.15)

include(cmake/doxygen.cmake)
include(cmake/osal.cmake)
include(cmake/platform.cmake)

if (NOT DEFINED PLATFORM)
    message(FATAL_ERROR "'PLATFORM' is not defined!")
endif ()

# Define application settings.
include(test/${APP}/${APP}.cmake)

# Setup platform toolchain file.
include(${platform_SOURCE_DIR}/app/${PLATFORM}/toolchain.cmake)

project(hal VERSION 1.0 LANGUAGES ASM C CXX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add conan files.
if (EXISTS ${CMAKE_BINARY_DIR}/conan_paths.cmake)
    include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
endif ()

# Project-wide compilation options.
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_subdirectory(${platform_SOURCE_DIR}/app ${platform_BINARY_DIR})
add_subdirectory(${osal_SOURCE_DIR}/lib ${osal_BINARY_DIR})
add_subdirectory(lib)
add_subdirectory(test/${APP})

# Generate documentation.
doxygen_addTarget(${CMAKE_CURRENT_SOURCE_DIR}/lib README.md)
