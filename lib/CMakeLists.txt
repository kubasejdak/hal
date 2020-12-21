cmake_minimum_required(VERSION 3.19)

# Project-wide compilation options.
add_compile_options(-Wall -Wextra -Wpedantic -Werror $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

add_library(hal EXCLUDE_FROM_ALL
    Hardware.cpp
    returnDevice.cpp
)

# Product has to be added first, because it configures the required boards and platform implementation.
add_subdirectory(product)
add_subdirectory(board)
add_subdirectory(drivers)
add_subdirectory(platform)
add_subdirectory(private)
add_subdirectory(public)
add_subdirectory(utils)

target_link_libraries(hal
    INTERFACE product
    PUBLIC hal-public
    PRIVATE hal-private
)

# Print HAL configuration.
message(STATUS "-------------------------------------------")
message(STATUS "HAL configuration:")
message(STATUS "  product  : ${HAL_PRODUCT}")
message(STATUS "  platform : ${HAL_PLATFORM}")
string(REPLACE ";" " " BOARDS_STR "${HAL_BOARDS}")
message(STATUS "  boards   : ${BOARDS_STR}")
message(STATUS "-------------------------------------------")
