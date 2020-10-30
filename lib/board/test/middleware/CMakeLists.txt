add_library(test-middleware-public INTERFACE)
add_library(test-middleware-private EXCLUDE_FROM_ALL
    BoardInit.cpp
)

target_include_directories(test-middleware-public
    INTERFACE include
)

target_link_libraries(test-middleware-public
    INTERFACE hal-public
)

target_link_libraries(test-middleware-private
    PUBLIC test-middleware-public hal-public
    PRIVATE hal-private
)
