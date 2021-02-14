add_library(sensor-sht3xdis EXCLUDE_FROM_ALL
    Sht3xDisHumidity.cpp
    Sht3xDisSensor.cpp
    Sht3xDisTemperature.cpp
)

target_include_directories(sensor-sht3xdis
    PUBLIC .
)

target_link_libraries(sensor-sht3xdis
    PUBLIC hal-public hal-private osal::cpp
    PRIVATE hal-logger
)
