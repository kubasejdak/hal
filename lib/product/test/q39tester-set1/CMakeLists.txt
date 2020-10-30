if (PLATFORM STREQUAL linux)
    set(HAL_PLATFORM linux PARENT_SCOPE)
    list(APPEND HAL_BOARDS sbc/raspberrypi)
endif ()

list(APPEND HAL_BOARDS test/q39tester-set1)
set(HAL_BOARDS ${HAL_BOARDS} PARENT_SCOPE)
