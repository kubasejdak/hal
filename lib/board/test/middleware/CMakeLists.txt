add_library(middleware-public INTERFACE)
add_library(middleware-private EXCLUDE_FROM_ALL)

if (PLATFORM STREQUAL linux OR PLATFORM STREQUAL linux-arm)
    target_sources(middleware-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(middleware-public
    INTERFACE include
)

target_link_libraries(middleware-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux OR PLATFORM STREQUAL linux-arm)
    target_link_libraries(middleware-private
        PUBLIC middleware-public hal-public
        PRIVATE hal-private
    )
endif ()
