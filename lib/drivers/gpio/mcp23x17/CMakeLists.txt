add_library(gpio-mcp23x17 EXCLUDE_FROM_ALL
    IMcp23x17.cpp
    Mcp23S17.cpp
    Mcp23017.cpp
)

target_include_directories(gpio-mcp23x17
    PUBLIC .
)

target_link_libraries(gpio-mcp23x17
    PUBLIC hal-public hal-private osal::cpp
    PRIVATE hal-logger
)
