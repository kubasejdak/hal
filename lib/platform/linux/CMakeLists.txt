set(DRIVERS
    gpio
    i2c
    uart
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
