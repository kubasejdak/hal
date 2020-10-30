add_library(test-uart-set1-public INTERFACE)
add_library(test-uart-set1-private EXCLUDE_FROM_ALL)

if (PLATFORM STREQUAL linux)
    target_sources(test-uart-set1-private
        PRIVATE BoardInit.cpp
    )
endif ()

target_include_directories(test-uart-set1-public
    INTERFACE include
)

target_link_libraries(test-uart-set1-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(test-uart-set1-private
        PUBLIC test-uart-set1-public hal-public
        PRIVATE hal-private uart
    )
endif ()
