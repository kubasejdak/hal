add_library(hal-sensor-sht3xdis EXCLUDE_FROM_ALL
    Sht3xDisHumidity.cpp
    Sht3xDisSensor.cpp
    Sht3xDisTemperature.cpp
)
add_library(hal::sensor-sht3xdis ALIAS hal-sensor-sht3xdis)

target_include_directories(hal-sensor-sht3xdis
    PUBLIC include
)

target_link_libraries(hal-sensor-sht3xdis
    PUBLIC hal::interfaces osal::cpp
    PRIVATE hal::logger
)
