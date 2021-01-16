set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)

add_library(stm32f4xx EXCLUDE_FROM_ALL
    startup_stm32f407xx.s
    system_stm32f4xx.c
    drivers/stm32f4xx_hal.c
    drivers/stm32f4xx_hal_cortex.c
    drivers/stm32f4xx_hal_gpio.c
    drivers/stm32f4xx_hal_rcc.c
    drivers/stm32f4xx_hal_uart.c
)

target_compile_definitions(stm32f4xx
    PUBLIC USE_HAL_DRIVER STM32F407xx
)

target_include_directories(stm32f4xx
    SYSTEM PUBLIC CMSIS/core
                  CMSIS/include
                  CMSIS/device
                  drivers
                  drivers/Legacy
                  .
)

target_compile_options(stm32f4xx
    PRIVATE -w
)
