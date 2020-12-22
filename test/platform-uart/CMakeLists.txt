# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appplatform-uart OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    ClientTests.cpp
)

target_include_directories(appplatform-uart
    PRIVATE .
)

conan_get(REQUIRES catch2/2.13.3 nlohmann_json/3.9.1)
find_package(Catch2 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/external/digestpp ${CMAKE_BINARY_DIR}/external/digestpp)

target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_DISABLE_EXCEPTIONS)

if (PLATFORM STREQUAL freertos-arm)
    # This is required, because FreeRTOS doesn't support signals or exceptions.
    target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_NO_POSIX_SIGNALS)
endif ()

target_link_libraries(appplatform-uart
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal nlohmann_json::nlohmann_json digestpp
)

# This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
if (DEFINED TEST_TAGS)
    target_compile_definitions(appplatform-uart
        PRIVATE TEST_TAGS="${TEST_TAGS}"
    )
endif ()

file(COPY ${CMAKE_CURRENT_LIST_DIR}/daemon DESTINATION ${CMAKE_BINARY_DIR}/bin)
