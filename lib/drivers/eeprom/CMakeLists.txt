set(DRIVERS
    generic
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
