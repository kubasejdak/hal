add_library(uart EXCLUDE_FROM_ALL
    LinuxUart.cpp
)

target_include_directories(uart
    PUBLIC .
)

target_link_libraries(uart
    PUBLIC hal-public osal::cpp
    PRIVATE hal-logger
)
