add_library(hal-eeprom-generic EXCLUDE_FROM_ALL
    GenericEeprom.cpp
)
add_library(hal::eeprom-generic ALIAS hal-eeprom-generic)

target_include_directories(hal-eeprom-generic
    PUBLIC .
)

target_link_libraries(hal-eeprom-generic
    PUBLIC hal::public hal::private osal::cpp
    PRIVATE hal::logger utils::bits
)
