# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appgpio-set1 OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    SingleBits.cpp
)

target_include_directories(appgpio-set1
    PRIVATE .
)

target_compile_features(appgpio-set1
    PRIVATE cxx_std_17
)

conan_get(REQUIRES Catch2/2.11.1@catchorg/stable)
find_package(Catch2 REQUIRED)

if (PLATFORM STREQUAL freertos-arm)
    # This is required, because FreeRTOS doesn't support signals or exceptions.
    target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_NO_POSIX_SIGNALS CATCH_CONFIG_DISABLE_EXCEPTIONS)
endif ()

target_link_libraries(appgpio-set1
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal
)

# This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
if (DEFINED TEST_TAGS)
    target_compile_definitions(appgpio-set1
        PRIVATE TEST_TAGS="${TEST_TAGS}"
    )
endif ()
