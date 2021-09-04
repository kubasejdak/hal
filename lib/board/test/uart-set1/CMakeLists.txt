add_library(hal-test-uart-set1 EXCLUDE_FROM_ALL)
add_library(hal::test-uart-set1 ALIAS hal-test-uart-set1)

if (PLATFORM STREQUAL linux)
    target_sources(hal-test-uart-set1
        PRIVATE BoardInit.cpp
    )
endif ()

target_include_directories(hal-test-uart-set1
    PUBLIC include
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(hal-test-uart-set1
        PUBLIC hal::interfaces
        PRIVATE hal::board-interface hal::uart
    )
endif ()
