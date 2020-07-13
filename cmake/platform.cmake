include(FetchContent)
FetchContent_Declare(platform
    GIT_REPOSITORY  https://gitlab.com/kubasejdak-libs/platform.git
    GIT_TAG         v1.1.2
)

FetchContent_GetProperties(platform)
if (NOT platform_POPULATED)
    FetchContent_Populate(platform)
endif ()

if (NOT DEFINED PLATFORM)
    message(FATAL_ERROR "'PLATFORM' is not defined!")
endif ()

# Setup platform toolchain file.
include(${platform_SOURCE_DIR}/app/${PLATFORM}/toolchain.cmake)
