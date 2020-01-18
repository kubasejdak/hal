add_library(raspberrypi3bplus_public INTERFACE)

list(APPEND SOURCES Board.cpp)

if (${PLATFORM} STREQUAL linux)
    list(APPEND SOURCES BoardLinux.cpp)
endif ()

add_library(raspberrypi3bplus_private EXCLUDE_FROM_ALL ${SOURCES})

target_include_directories(raspberrypi3bplus_public
    INTERFACE include/public/
)
target_include_directories(raspberrypi3bplus_private
    PUBLIC include/private/
)

if (${PLATFORM} STREQUAL linux)
    target_link_libraries(raspberrypi3bplus_private
        PUBLIC raspberrypi3bplus_public hal_public hal_private
        PRIVATE gpio
    )
endif ()
