add_library(hal-test-platform-gpio EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(hal::test-platform-gpio ALIAS hal-test-platform-gpio)
add_library(hal-test-platform-gpio-config INTERFACE)
add_library(hal::test-platform-gpio-config ALIAS hal-test-platform-gpio-config)

target_include_directories(hal-test-platform-gpio
    PUBLIC include
)

target_include_directories(hal-test-platform-gpio-config
    INTERFACE config
)

target_link_libraries(hal-test-platform-gpio
    PUBLIC hal::interfaces
    PRIVATE hal::board-interface hal::product-config
)

target_link_libraries(hal-test-platform-gpio-config
    INTERFACE utils::types
)
