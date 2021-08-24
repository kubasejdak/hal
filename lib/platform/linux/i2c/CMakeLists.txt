add_library(hal-i2c EXCLUDE_FROM_ALL
    LinuxI2c.cpp
)
add_library(hal::i2c ALIAS hal-i2c)

target_include_directories(hal-i2c
    PUBLIC .
)

target_link_libraries(hal-i2c
    PUBLIC hal-public hal-private osal::cpp
)
