add_executable(platform-gpio
    appMain.cpp
    GpioPorts.cpp
    SingleBits.cpp
)

target_compile_definitions(platform-gpio
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(platform-gpio
    PRIVATE hal osal::cpp platform::init platform::main
)
