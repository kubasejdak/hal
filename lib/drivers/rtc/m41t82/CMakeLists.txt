add_library(rtc-m41t82 EXCLUDE_FROM_ALL
    M41T82.cpp
)

target_include_directories(rtc-m41t82
    PUBLIC .
)

target_link_libraries(rtc-m41t82
    PUBLIC hal-public hal-private
    PRIVATE hal-logger osal::cpp
)
