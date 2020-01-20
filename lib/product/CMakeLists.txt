if (NOT DEFINED HAL_PRODUCT)
    message(FATAL_ERROR "'HAL_PRODUCT' is not defined!")
endif ()

add_library(product EXCLUDE_FROM_ALL
    ${HAL_PRODUCT}/Hardware.cpp
)

target_include_directories(product
    PUBLIC ${HAL_PRODUCT}/include
)

target_link_libraries(product
    PUBLIC board-public
    PRIVATE hal-public board-private
)

add_subdirectory(${HAL_PRODUCT})

if (NOT HAL_PLATFORM)
    message(FATAL_ERROR "'HAL_PLATFORM' is not defined by the '${HAL_PRODUCT}' product!")
endif ()

if (NOT HAL_BOARDS)
    message(FATAL_ERROR "'HAL_BOARDS' is not defined by the '${HAL_PRODUCT}' product!")
endif ()

# Make HAL_PLATFORM and HAL_BOARDS, set by the concrete product, visible in the HAL root directory.
set(HAL_PLATFORM ${HAL_PLATFORM} PARENT_SCOPE)
set(HAL_BOARDS ${HAL_BOARDS} PARENT_SCOPE)
