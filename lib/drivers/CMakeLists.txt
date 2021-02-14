set(MODULES
    eeprom
    gpio
    rtc
    sensor
)

foreach (MODULE IN LISTS MODULES)
    add_subdirectory(${MODULE})
endforeach ()
