# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appgpio-set1 OBJECT EXCLUDE_FROM_ALL
    4PinPorts.cpp
    appMain.cpp
    MultithreadGpio.cpp
    SingleBits.cpp
)

target_include_directories(appgpio-set1
    PRIVATE .
)

target_compile_definitions(appgpio-set1
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(appgpio-set1
    PUBLIC platform-init
    PRIVATE osal::cpp hal
)
