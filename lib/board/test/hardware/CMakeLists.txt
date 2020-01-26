add_library(hardware-public INTERFACE)
add_library(hardware-private EXCLUDE_FROM_ALL)

if (${PLATFORM} STREQUAL linux)
    target_sources(hardware-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(hardware-public
    INTERFACE include
)

if (${PLATFORM} STREQUAL linux)
    target_link_libraries(hardware-private
        PUBLIC hardware-public hal-public hal-private
        PRIVATE gpio
    )
endif ()
