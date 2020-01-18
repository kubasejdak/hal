set(DRIVERS
    gpio
    uart
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
