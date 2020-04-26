add_library(gpio EXCLUDE_FROM_ALL
    LinuxGpio.cpp
)

target_include_directories(gpio
    PUBLIC .
)

conan_add_remote(NAME kubasejdak URL https://api.bintray.com/conan/kubasejdak/public-conan)
conan_get(REQUIRES libgpiod/1.4.3@kubasejdak/stable fmt/6.2.0 OPTIONS fmt:header_only=True)
find_package(libgpiod REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(gpio
    PUBLIC hal-public hal-private libgpiod::libgpiod fmt::fmt utils
)
