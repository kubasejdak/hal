add_library(test-device INTERFACE)

target_include_directories(test-device
    INTERFACE .
)

target_link_libraries(test-device
    INTERFACE hal-public
)
