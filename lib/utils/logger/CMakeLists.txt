add_library(hal-logger INTERFACE)

target_include_directories(hal-logger
    INTERFACE include
)

target_link_libraries(hal-logger
    INTERFACE utils::logger
)
