add_library(uart EXCLUDE_FROM_ALL
    TtyUart.cpp
)

target_include_directories(uart
    PUBLIC .
)

target_link_libraries(uart
    PUBLIC hal_public
    PRIVATE stdc++fs
)
