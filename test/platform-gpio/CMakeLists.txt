# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appplatform-gpio OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    GpioPorts.cpp
    SingleBits.cpp
)

target_include_directories(appplatform-gpio
    PRIVATE .
)

target_compile_definitions(appplatform-gpio
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(appplatform-gpio
    PUBLIC platform-init
    PRIVATE osal::cpp hal
)
