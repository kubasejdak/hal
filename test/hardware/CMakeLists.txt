add_subdirectory(init/${PLATFORM})

# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(apphardware OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    initialization.cpp
)

target_include_directories(apphardware
    PRIVATE .
)

target_compile_features(apphardware
    PRIVATE cxx_std_17
)

find_package(Catch2 REQUIRED)

if (PLATFORM STREQUAL "freertos-arm")
    # This is required, because FreeRTOS doesn't support signals or exceptions.
    target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_NO_POSIX_SIGNALS CATCH_CONFIG_DISABLE_EXCEPTIONS)
endif ()

target_link_libraries(apphardware
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 hal
)

# This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
if (DEFINED TEST_TAGS)
    target_compile_definitions(apphardware
        PRIVATE TEST_TAGS="${TEST_TAGS}"
    )
endif ()
