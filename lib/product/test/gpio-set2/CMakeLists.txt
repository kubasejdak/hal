if (PLATFORM STREQUAL linux OR PLATFORM STREQUAL linux-arm)
    set(HAL_PLATFORM linux PARENT_SCOPE)
endif ()

set(HAL_BOARDS
    test/gpio-set2
PARENT_SCOPE)
