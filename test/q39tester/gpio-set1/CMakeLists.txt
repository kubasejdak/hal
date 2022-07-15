add_executable(gpio-set1
    4PinPorts.cpp
    appMain.cpp
    MultithreadGpio.cpp
    SingleBits.cpp
)

target_compile_definitions(gpio-set1
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(gpio-set1
    PRIVATE hal osal::cpp platform::init platform::main
)
