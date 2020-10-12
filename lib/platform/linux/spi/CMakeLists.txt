add_library(spi EXCLUDE_FROM_ALL
    LinuxSpi.cpp
)

target_include_directories(spi
    PUBLIC .
)

target_link_libraries(spi
    PUBLIC hal-public hal-private osal::cpp
)
