add_library(gpio EXCLUDE_FROM_ALL
    LinuxGpio.cpp
)

target_include_directories(gpio
    PUBLIC .
)

conan_add_remote(NAME ecashptyltd URL https://api.bintray.com/conan/ecashptyltd/public-conan)
conan_get(libgpiod/1.2.1@ecashptyltd/stable)
find_package(libgpiod REQUIRED)

target_link_libraries(gpio
    PUBLIC hal-public hal-private libgpiod::libgpiod utils
)
