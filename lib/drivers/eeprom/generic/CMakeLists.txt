add_library(hal-eeprom-generic EXCLUDE_FROM_ALL
    GenericEeprom.cpp
)
add_library(hal::eeprom-generic ALIAS hal-eeprom-generic)

target_include_directories(hal-eeprom-generic
    PUBLIC include
)

target_link_libraries(hal-eeprom-generic
    PUBLIC hal::interfaces osal::cpp
    PRIVATE hal::logger utils::bits
)
