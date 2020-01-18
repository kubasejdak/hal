add_library(product EXCLUDE_FROM_ALL
    ${PRODUCT}/Hardware.cpp
)

target_include_directories(product
    PUBLIC ${PRODUCT}/include/
)

target_link_libraries(product
    PUBLIC board_public
    PRIVATE hal_public board_private
)

add_subdirectory(${PRODUCT})

if (NOT PLATFORM)
    message(FATAL_ERROR "'PLATFORM' is not defined by the '${PRODUCT}' product!")
endif ()

if (NOT BOARDS)
    message(FATAL_ERROR "'BOARDS' is not defined by the '${PRODUCT}' product!")
endif ()

# Make PLATFORM and BOARDS, set by the concrete product, visible in the HAL root directory.
set(PLATFORM ${PLATFORM} PARENT_SCOPE)
set(BOARDS ${BOARDS} PARENT_SCOPE)
