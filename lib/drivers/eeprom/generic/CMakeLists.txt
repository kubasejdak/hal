add_library(eeprom-generic EXCLUDE_FROM_ALL
    GenericEeprom.cpp
)

target_include_directories(eeprom-generic
    PUBLIC .
)

target_link_libraries(eeprom-generic
    PUBLIC hal-public hal-private osal::cpp
    PRIVATE hal-logger utils::bits
)
