add_library(raspberrypi-public INTERFACE)
add_library(raspberrypi-private EXCLUDE_FROM_ALL)

if (PLATFORM STREQUAL linux)
    target_sources(raspberrypi-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(raspberrypi-public
    INTERFACE include
)

target_link_libraries(raspberrypi-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(raspberrypi-private
        PUBLIC raspberrypi-public hal-public
        PRIVATE hal-private gpio
    )
endif ()
