add_library(hal-logger INTERFACE)
add_library(hal::logger ALIAS hal-logger)

target_include_directories(hal-logger
    INTERFACE include
)

target_link_libraries(hal-logger
    INTERFACE utils::logger
)
