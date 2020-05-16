if (PLATFORM STREQUAL linux)
    set(HAL_PLATFORM linux PARENT_SCOPE)
endif ()

set(HAL_BOARDS
    sbc/raspberrypi
    test/gpio-set2
PARENT_SCOPE)
