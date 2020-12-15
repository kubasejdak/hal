add_library(rtc-m41t82
    M41T82.cpp
)

target_include_directories(rtc-m41t82
    PUBLIC .
)

target_link_libraries(rtc-m41t82
    PUBLIC hal-public hal-private osal::cpp
    PRIVATE hal-logger
)
