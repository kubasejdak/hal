add_library(i2c EXCLUDE_FROM_ALL
    LinuxI2c.cpp
)

target_include_directories(i2c
    PUBLIC .
)

target_link_libraries(i2c
    PUBLIC hal-public hal-private osal::cpp
)
