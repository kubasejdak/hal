add_executable(middleware
    appMain.cpp
    Error.cpp
    ScopedDevice.cpp
    initialization.cpp
)

target_compile_definitions(middleware
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(middleware
    PRIVATE hal hal::test-device osal::cpp platform::init platform::main
)
