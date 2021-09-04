add_library(hal-sbc-raspberrypi EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(hal::sbc-raspberrypi ALIAS hal-sbc-raspberrypi)
add_library(hal-sbc-raspberrypi-config INTERFACE)
add_library(hal::sbc-raspberrypi-config ALIAS hal-sbc-raspberrypi-config)

target_include_directories(hal-sbc-raspberrypi
    PUBLIC include
)

target_include_directories(hal-sbc-raspberrypi-config
    INTERFACE config
)

target_link_libraries(hal-sbc-raspberrypi
    PUBLIC hal::interfaces
    PRIVATE hal::board-interface hal::gpio hal::spi hal::i2c
)

target_link_libraries(hal-sbc-raspberrypi-config
    INTERFACE utils::types
)
