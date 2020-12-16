set(MODULES
    eeprom
    gpio
    rtc
)

foreach (MODULE IN LISTS MODULES)
    add_subdirectory(${MODULE})
endforeach ()
