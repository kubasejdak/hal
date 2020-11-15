# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appeeprom-set1 OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    EepromTests.cpp
)

target_include_directories(appeeprom-set1
    PRIVATE .
)

conan_get(REQUIRES catch2/2.13.2)
find_package(Catch2 REQUIRED)

if (PLATFORM STREQUAL freertos-arm)
    # This is required, because FreeRTOS doesn't support signals or exceptions.
    target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_NO_POSIX_SIGNALS CATCH_CONFIG_DISABLE_EXCEPTIONS)
endif ()

target_link_libraries(appeeprom-set1
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal
)

# This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
if (DEFINED TEST_TAGS)
    target_compile_definitions(appeeprom-set1
        PRIVATE TEST_TAGS="${TEST_TAGS}"
    )
endif ()