# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appuart-set1 OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    ClientTests.cpp
)

target_include_directories(appuart-set1
    PRIVATE .
)

conan_get(REQUIRES catch2/2.13.0 rapidjson/1.1.0)
find_package(Catch2 REQUIRED)
find_package(RapidJSON REQUIRED)

add_subdirectory(${CMAKE_SOURCE_DIR}/external/digestpp ${CMAKE_BINARY_DIR}/external/digestpp)

if (PLATFORM STREQUAL freertos-arm)
    # This is required, because FreeRTOS doesn't support signals or exceptions.
    target_compile_definitions(Catch2::Catch2 INTERFACE CATCH_CONFIG_NO_POSIX_SIGNALS CATCH_CONFIG_DISABLE_EXCEPTIONS)
endif ()

target_link_libraries(appuart-set1
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal RapidJSON::RapidJSON digestpp
)

# This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
if (DEFINED TEST_TAGS)
    target_compile_definitions(appuart-set1
        PRIVATE TEST_TAGS="${TEST_TAGS}"
    )
endif ()

file(COPY ${CMAKE_CURRENT_LIST_DIR}/daemon DESTINATION ${CMAKE_BINARY_DIR}/bin)
