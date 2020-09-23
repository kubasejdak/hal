add_library(test-middleware-public INTERFACE)
add_library(test-middleware-private EXCLUDE_FROM_ALL)

if (PLATFORM STREQUAL linux)
    target_sources(test-middleware-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(test-middleware-public
    INTERFACE include
)

target_link_libraries(test-middleware-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(test-middleware-private
        PUBLIC test-middleware-public hal-public
        PRIVATE hal-private
    )
endif ()
