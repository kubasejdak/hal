add_library(hal-test-device INTERFACE)
add_library(hal::test-device ALIAS hal-test-device)

target_include_directories(hal-test-device
    INTERFACE include
)

target_link_libraries(hal-test-device
    INTERFACE hal::interfaces
)
