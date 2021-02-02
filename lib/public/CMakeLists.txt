add_library(hal-public EXCLUDE_FROM_ALL
    Device.cpp
    Error.cpp
    IEeprom.cpp
    IHumiditySensor.cpp
    IRtc.cpp
    ITemperatureSensor.cpp
    IUart.cpp
)

target_include_directories(hal-public
    PUBLIC include
)

target_link_libraries(hal-public
    PUBLIC osal::cpp utils::registry hal-logger
)
