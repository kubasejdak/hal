add_library(hal-gpio-mcp23x17 EXCLUDE_FROM_ALL
    IMcp23x17.cpp
    Mcp23S17.cpp
    Mcp23017.cpp
)
add_library(hal::gpio-mcp23x17 ALIAS hal-gpio-mcp23x17)

target_include_directories(hal-gpio-mcp23x17
    PUBLIC .
)

target_link_libraries(hal-gpio-mcp23x17
    PUBLIC hal::public hal::private osal::cpp
    PRIVATE hal::logger
)
