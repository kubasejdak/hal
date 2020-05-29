add_library(gpio-set1-public INTERFACE)
add_library(gpio-set1-private EXCLUDE_FROM_ALL)
add_library(gpio-set1-config INTERFACE)

if (PLATFORM STREQUAL linux)
    target_sources(gpio-set1-private
        PRIVATE BoardLinux.cpp
    )
endif ()

target_include_directories(gpio-set1-public
    INTERFACE include
)

target_include_directories(gpio-set1-config
    INTERFACE config
)

target_link_libraries(gpio-set1-public
    INTERFACE hal-public
)

if (PLATFORM STREQUAL linux)
    target_link_libraries(gpio-set1-private
        PUBLIC gpio-set1-public hal-public
        PRIVATE hal-private product-config
    )
endif ()

target_link_libraries(gpio-set1-config
    INTERFACE utils::property
)
