set(DRIVERS
    gpio
    i2c
    spi
    uart
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
