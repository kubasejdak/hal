set(LINUX_ARM_TOOLCHAIN_PATH        "/opt/toolchains/gcc-arm-11.2-2022.02-x86_64-arm-none-linux-gnueabihf/bin/")
set(BAREMETAL_ARM_TOOLCHAIN_PATH    "/opt/toolchains/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin/")

set(FREERTOS_VERSION                freertos-10.2.1)
set(FREERTOS_PORTABLE               ARM_CM4F)

if (PLATFORM STREQUAL freertos-arm)
    set(APP_C_FLAGS                 "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb" CACHE INTERNAL "")
    set(APP_CXX_FLAGS               "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb" CACHE INTERNAL "")
endif ()

set(APP_CXX_FLAGS                   "${APP_CXX_FLAGS} -fno-exceptions" CACHE INTERNAL "")

set(HAL_PRODUCT                     test/platform-uart)
