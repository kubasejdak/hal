add_library(test-platform-gpio-public INTERFACE)
add_library(test-platform-gpio-private EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(test-platform-gpio-config INTERFACE)

target_include_directories(test-platform-gpio-public
    INTERFACE include
)

target_include_directories(test-platform-gpio-config
    INTERFACE config
)

target_link_libraries(test-platform-gpio-public
    INTERFACE hal-public
)

target_link_libraries(test-platform-gpio-private
    PUBLIC test-platform-gpio-public hal-public
    PRIVATE hal-private product-config
)

target_link_libraries(test-platform-gpio-config
    INTERFACE utils::property
)
