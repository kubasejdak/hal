include(FetchContent)
FetchContent_Declare(platform
    GIT_REPOSITORY  https://gitlab.com/kubasejdak-libs/platform.git
    GIT_TAG         origin/master
)

FetchContent_GetProperties(platform)
if (NOT platform_POPULATED)
    FetchContent_Populate(platform)
endif ()

if (NOT DEFINED PLATFORM)
    set(DEFAULT_PLATFORM    linux)
    message(STATUS "'PLATFORM' is not defined. Using '${DEFAULT_PLATFORM}'")
    set(PLATFORM            ${DEFAULT_PLATFORM})
endif ()

# Setup platform toolchain file.
include(${platform_SOURCE_DIR}/app/${PLATFORM}/toolchain.cmake)
