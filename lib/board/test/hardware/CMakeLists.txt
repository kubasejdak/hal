add_library(hardware-public INTERFACE)

list(APPEND SOURCES Board.cpp)

if (${PLATFORM} STREQUAL linux)
    list(APPEND SOURCES BoardLinux.cpp)
endif ()

add_library(hardware-private EXCLUDE_FROM_ALL ${SOURCES})

target_include_directories(hardware-public
    INTERFACE include/public
)
target_include_directories(hardware-private
    PUBLIC include/private
)

if (${PLATFORM} STREQUAL linux)
    target_link_libraries(hardware-private
        PUBLIC hardware-public hal-public hal-private
        PRIVATE gpio
    )
endif ()
