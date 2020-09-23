add_library(sbc-raspberrypi-public INTERFACE)
add_library(sbc-raspberrypi-private EXCLUDE_FROM_ALL)
add_library(sbc-raspberrypi-config INTERFACE)

if (PLATFORM STREQUAL linux)
    target_sources(sbc-raspberrypi-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(sbc-raspberrypi-public
    INTERFACE include
)

target_include_directories(sbc-raspberrypi-config
    INTERFACE config
)

target_link_libraries(sbc-raspberrypi-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(sbc-raspberrypi-private
        PUBLIC sbc-raspberrypi-public hal-public
        PRIVATE hal-private product-config gpio
    )
endif ()

target_link_libraries(sbc-raspberrypi-config
    INTERFACE utils::property
)
