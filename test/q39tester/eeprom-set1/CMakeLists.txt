add_executable(eeprom-set1
    appMain.cpp
    EepromTests.cpp
)

target_compile_definitions(eeprom-set1
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(eeprom-set1
    PRIVATE hal osal::cpp platform::init platform::main
)
