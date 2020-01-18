add_library(gpio INTERFACE)

target_include_directories(gpio
    INTERFACE .
)

target_link_libraries(gpio
    INTERFACE hal_public hal_private stdc++fs
)
