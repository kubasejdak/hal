set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)

add_library(stm32f4 EXCLUDE_FROM_ALL
    CMSIS/system_stm32f4xx.c
    CMSIS/startup_stm32f40_41xxx.s
    drivers/stm32f4xx_gpio.c
    drivers/stm32f4xx_rcc.c
    drivers/stm32f4xx_usart.c
)

target_compile_definitions(stm32f4
    PUBLIC USE_STDPERIPH_DRIVER
)

target_include_directories(stm32f4
    SYSTEM PUBLIC CMSIS drivers
)

target_compile_options(stm32f4
    PRIVATE -w
)
