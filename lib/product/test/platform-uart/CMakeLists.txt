if (PLATFORM STREQUAL linux)
    set(HAL_PLATFORM linux PARENT_SCOPE)
endif ()

set(HAL_BOARDS
    test/uart-set1
PARENT_SCOPE)
