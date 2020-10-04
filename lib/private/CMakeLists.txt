add_library(hal-private INTERFACE)

target_include_directories(hal-private
    INTERFACE include
)

target_link_libraries(hal-private
    INTERFACE hal-public utils::registry
)
