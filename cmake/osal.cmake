include(FetchContent)
FetchContent_Declare(
    osal
    GIT_REPOSITORY  https://github.com/kubasejdak/osal.git
    GIT_TAG         master
)

FetchContent_GetProperties(osal)
if (NOT osal_POPULATED)
    FetchContent_Populate(osal)
endif ()
