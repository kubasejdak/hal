include(FetchContent)
FetchContent_Declare(platform
    GIT_REPOSITORY  https://gitlab.com/kubasejdak-libs/platform.git
    GIT_TAG         origin/master
)

FetchContent_GetProperties(platform)
if (NOT platform_POPULATED)
    FetchContent_Populate(platform)
endif ()
