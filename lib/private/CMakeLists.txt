add_library(hal-private INTERFACE)

target_include_directories(hal-private
    INTERFACE include
)

find_package(Boost REQUIRED)

target_link_libraries(hal-private
    INTERFACE hal-public Boost::Boost
)
