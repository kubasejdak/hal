add_library(test-q39tester-set1-public INTERFACE)
add_library(test-q39tester-set1-private EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(test-q39tester-set1-config INTERFACE)

target_include_directories(test-q39tester-set1-public
    INTERFACE include
)

target_include_directories(test-q39tester-set1-config
    INTERFACE config
)

target_link_libraries(test-q39tester-set1-public
    INTERFACE hal-public
)

target_link_libraries(test-q39tester-set1-private
    PUBLIC test-q39tester-set1-public hal-public
    PRIVATE hal-private product-config gpio-mcp23x17 eeprom-generic rtc-m41t82 sensor-sht3xdis
)

target_link_libraries(test-q39tester-set1-config
    INTERFACE utils::types
)
