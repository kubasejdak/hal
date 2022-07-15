add_executable(platform-uart
    appMain.cpp
    ClientTests.cpp
)

add_subdirectory(${CMAKE_SOURCE_DIR}/external/hash ${CMAKE_BINARY_DIR}/external/hash)

target_compile_definitions(platform-uart
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}"> CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS> JSON_NOEXCEPTION
)

target_link_libraries(platform-uart
    PRIVATE hal hash osal::cpp platform::init platform::main
)

file(COPY ${CMAKE_CURRENT_LIST_DIR}/daemon DESTINATION ${CMAKE_BINARY_DIR}/bin)
