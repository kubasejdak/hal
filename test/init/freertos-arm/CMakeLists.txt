add_subdirectory(${CMAKE_SOURCE_DIR}/external/stm32f4 ${CMAKE_BINARY_DIR}/external/stm32f4)

add_library(platform-init EXCLUDE_FROM_ALL
    platformInit.cpp
)

target_include_directories(platform-init
    PUBLIC ..
)

target_link_libraries(platform-init
    PRIVATE stm32f4
)

target_link_options(platform-init
    PUBLIC -T ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4discovery.ld
)

# This target is required by the freertos-arm platform.
add_library(freertos-config INTERFACE)

target_include_directories(freertos-config
    INTERFACE .
)

target_link_libraries(freertos-config
    INTERFACE stm32f4
)
