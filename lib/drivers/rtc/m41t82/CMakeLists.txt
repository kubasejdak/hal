add_library(hal-rtc-m41t82 EXCLUDE_FROM_ALL
    M41T82.cpp
)
add_library(hal::rtc-m41t82 ALIAS hal-rtc-m41t82)

target_include_directories(hal-rtc-m41t82
    PUBLIC .
)

target_link_libraries(hal-rtc-m41t82
    PUBLIC hal::public hal::private
    PRIVATE hal::logger osal::cpp
)
