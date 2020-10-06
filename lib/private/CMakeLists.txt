add_library(hal-private
    II2c.cpp
)

target_include_directories(hal-private
    PUBLIC include
)

target_link_libraries(hal-private
    PUBLIC hal-public utils::registry osal::cpp
    PRIVATE hal-logger
)
