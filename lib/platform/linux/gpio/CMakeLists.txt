add_library(gpio EXCLUDE_FROM_ALL
    LinuxGpio.cpp
)

target_include_directories(gpio
    PUBLIC .
)

conan_add_remote(NAME kubasejdak URL https://gitlab.com/api/v4/projects/17978160/packages/conan)
conan_get(REQUIRES libgpiod/1.4.3@kubasejdak/stable fmt/7.1.3 OPTIONS libgpiod:build_cxx=False fmt:header_only=True)
find_package(libgpiod REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(gpio
    PUBLIC hal-public hal-private libgpiod::libgpiod fmt::fmt
)
