set(MODULES
    gpio
)

foreach (MODULE IN LISTS MODULES)
    add_subdirectory(${MODULE})
endforeach ()
