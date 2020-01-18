add_library(hal_public EXCLUDE_FROM_ALL
    Device.cpp
    Error.cpp
    IUart.cpp
)

target_include_directories(hal_public
    PUBLIC include/
)
