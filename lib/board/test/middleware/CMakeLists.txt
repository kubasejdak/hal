add_library(hal-test-middleware EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(hal::test-middleware ALIAS hal-test-middleware)

target_include_directories(hal-test-middleware
    PUBLIC include
)

target_link_libraries(hal-test-middleware
    PUBLIC hal::interfaces
    PRIVATE hal::board-interface hal::test-device
)
