add_library(gpio-set2-public INTERFACE)
add_library(gpio-set2-private EXCLUDE_FROM_ALL)
add_library(gpio-set2-config INTERFACE)

if (PLATFORM STREQUAL linux)
    target_sources(gpio-set2-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(gpio-set2-public
    INTERFACE include
)

target_include_directories(gpio-set2-config
    INTERFACE config
)

target_link_libraries(gpio-set2-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(gpio-set2-private
        PUBLIC gpio-set2-public hal-public
        PRIVATE hal-private product-config
    )
endif ()

target_link_libraries(gpio-set2-config
    INTERFACE utils::property
)
