set(DRIVERS
    test
)

foreach (DRIVER IN LISTS DRIVERS)
    add_subdirectory(${DRIVER})
endforeach ()
