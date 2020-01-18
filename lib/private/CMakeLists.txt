add_library(hal_private INTERFACE)

target_include_directories(hal_private
    INTERFACE include/
)

target_link_libraries(hal_private
    INTERFACE hal_public Boost::boost
)
