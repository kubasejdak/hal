add_library(hal-test-q39tester-set1 EXCLUDE_FROM_ALL
    BoardInit.cpp
)
add_library(hal::test-q39tester-set1 ALIAS hal-test-q39tester-set1)
add_library(hal-test-q39tester-set1-config INTERFACE)
add_library(hal::test-q39tester-set1-config ALIAS hal-test-q39tester-set1-config)

target_include_directories(hal-test-q39tester-set1
    PUBLIC include
)

target_include_directories(hal-test-q39tester-set1-config
    INTERFACE config
)

target_link_libraries(hal-test-q39tester-set1
    PUBLIC hal::interfaces
    PRIVATE hal::board-interface hal::product-config hal::gpio-mcp23x17 hal::eeprom-generic hal::rtc-m41t82 hal::sensor-sht3xdis
)

target_link_libraries(hal-test-q39tester-set1-config
    INTERFACE utils::types
)
