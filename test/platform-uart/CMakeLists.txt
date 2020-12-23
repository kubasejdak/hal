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

add_subdirectory(${CMAKE_SOURCE_DIR}/external/hash ${CMAKE_BINARY_DIR}/external/hash)

target_compile_definitions(appplatform-uart
    # This is required to selectively run Catch2 tests using tags on platforms, where cmd line arguments are not available.
    PRIVATE $<$<BOOL:${TEST_TAGS}>:TEST_TAGS="${TEST_TAGS}">
)

target_compile_definitions(Catch2::Catch2
    INTERFACE CATCH_CONFIG_DISABLE_EXCEPTIONS $<$<STREQUAL:${PLATFORM},freertos-arm>:CATCH_CONFIG_NO_POSIX_SIGNALS>
)

target_compile_definitions(nlohmann_json::nlohmann_json
    INTERFACE JSON_NOEXCEPTION
)

target_link_libraries(appplatform-uart
    PUBLIC platform-init
    PRIVATE Catch2::Catch2 osal::cpp hal nlohmann_json::nlohmann_json hash
)

file(COPY ${CMAKE_CURRENT_LIST_DIR}/daemon DESTINATION ${CMAKE_BINARY_DIR}/bin)
