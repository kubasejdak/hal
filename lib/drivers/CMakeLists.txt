set(MODULES
    eeprom
    gpio
)

foreach (MODULE IN LISTS MODULES)
    add_subdirectory(${MODULE})
endforeach ()
