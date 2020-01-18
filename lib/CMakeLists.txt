if (NOT DEFINED PRODUCT)
    message(FATAL_ERROR "'PRODUCT' is not defined!")
endif ()

add_library(hal EXCLUDE_FROM_ALL
    Hardware.cpp
    ReturnDevice.cpp
)

# Product has to be added first, because it configures the required boards and platform implementation.
add_subdirectory(product)
add_subdirectory(board)
add_subdirectory(platform)
add_subdirectory(private)
add_subdirectory(public)

target_link_libraries(hal
    INTERFACE product
    PUBLIC hal_public
    PRIVATE hal_private
)

# Print HAL configuration.
utils_log("-------------------------------------------")
utils_log("HAL configuration:")
utils_log("  product  : ${PRODUCT}")
utils_log("  platform : ${PLATFORM}")
string(REPLACE ";" " " BOARDS_STR "${BOARDS}")
utils_log("  boards   : ${BOARDS_STR}")
utils_log("-------------------------------------------")
