set(DRIVERS
    sht3xdis
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
