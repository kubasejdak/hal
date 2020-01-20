add_library(gpio INTERFACE)

target_include_directories(gpio
    INTERFACE .
)

target_link_libraries(gpio
    INTERFACE hal-public hal-private stdc++fs
)
