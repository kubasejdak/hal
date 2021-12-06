# Object library type is used to prevent compiler from removing unit test functions from the binary.
add_library(appplatform-uart OBJECT EXCLUDE_FROM_ALL
    appMain.cpp
    ClientTests.cpp
)

target_include_directories(appplatform-uart
    PRIVATE .
)

add_subdirectory(${CMAKE_SOURCE_DIR}/external/hash ${CMAKE_BINARY_DIR}/external/hash)

target_compile_definitions(appplatform-uart
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS> JSON_NOEXCEPTION
)

target_link_libraries(appplatform-uart
    PUBLIC platform-init
    PRIVATE osal::cpp hal hash
)

file(COPY ${CMAKE_CURRENT_LIST_DIR}/daemon DESTINATION ${CMAKE_BINARY_DIR}/bin)
