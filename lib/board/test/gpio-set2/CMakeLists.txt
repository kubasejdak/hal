add_library(test-gpio-set2-public INTERFACE)
add_library(test-gpio-set2-private EXCLUDE_FROM_ALL)
add_library(test-gpio-set2-config INTERFACE)

if (PLATFORM STREQUAL linux)
    target_sources(test-gpio-set2-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(test-gpio-set2-public
    INTERFACE include
)

target_include_directories(test-gpio-set2-config
    INTERFACE config
)

target_link_libraries(test-gpio-set2-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(test-gpio-set2-private
        PUBLIC test-gpio-set2-public hal-public
        PRIVATE hal-private product-config
    )
endif ()

target_link_libraries(test-gpio-set2-config
    INTERFACE utils::property
)
