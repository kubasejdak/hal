if (NOT DEFINED HAL_PRODUCT)
    message(FATAL_ERROR "'HAL_PRODUCT' is not defined!")
endif ()

add_library(hal-product EXCLUDE_FROM_ALL
    Hardware.cpp
    returnDevice.cpp
    ${HAL_PRODUCT}/Hardware.cpp
)
add_library(hal::product ALIAS hal-product)

add_library(hal-product-config INTERFACE)
add_library(hal::product-config ALIAS hal-product-config)

target_include_directories(hal-product
    PUBLIC include ${HAL_PRODUCT}/include
)

target_include_directories(hal-product-config
    INTERFACE ${HAL_PRODUCT}/config
)

target_link_libraries(hal-product
    PUBLIC hal::board
    PRIVATE hal::interfaces hal::board-interface
)

target_link_libraries(hal-product-config
    INTERFACE hal::board-config
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
