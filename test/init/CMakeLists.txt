add_library(platform-init EXCLUDE_FROM_ALL)
add_library(platform::init ALIAS platform-init)

target_include_directories(platform-init
    PUBLIC include
)

add_subdirectory(${PLATFORM})
