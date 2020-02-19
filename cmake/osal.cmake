include(FetchContent)
FetchContent_Declare(osal
    GIT_REPOSITORY  https://gitlab.com/kubasejdak-libs/osal.git
    GIT_TAG         origin/master
)

FetchContent_GetProperties(osal)
if (NOT osal_POPULATED)
    FetchContent_Populate(osal)
endif ()
