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

conan_get(REQUIRES catch2/2.13.3)
find_package(Catch2 REQUIRED)

target_compile_definitions(appgpio-set1
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}">
)

target_compile_definitions(Catch2::Catch2
    INTERFACE CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_link_libraries(appgpio-set1
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal
)
