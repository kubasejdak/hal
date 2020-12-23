set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)

add_library(hash EXCLUDE_FROM_ALL
    sha256.cpp
)

target_include_directories(hash
    SYSTEM INTERFACE include
    PRIVATE include/hash
)

target_compile_options(hash
    PRIVATE -w
)
