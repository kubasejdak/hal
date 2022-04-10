option(USE_CONAN "Automatically use conan to install required dependencies" OFF)

function(conan_init_wrapper)
    include(FetchContent)
    FetchContent_Declare(conan-wrapper
        GIT_REPOSITORY  https://github.com/conan-io/cmake-conan.git
        GIT_TAG         0.18.1
    )

    FetchContent_GetProperties(conan-wrapper)
    if (NOT conan-wrapper_POPULATED)
        FetchContent_Populate(conan-wrapper)
    endif ()

    include(${conan-wrapper_SOURCE_DIR}/conan.cmake)
endfunction (conan_init_wrapper)

function(conan_init_profile)
    # Value for settings.os.
    set(CONAN_OS                ${CMAKE_SYSTEM_NAME})

    # Value for settings.os_build.
    set(CONAN_HOST_OS           ${CMAKE_HOST_SYSTEM_NAME})

    # Value for settings.arch.
    if (NOT DEFINED CONAN_ARCH)
        if (PLATFORM STREQUAL linux)
            if (CMAKE_CROSSCOMPILING)
                set(CONAN_ARCH  armv8_32)
            else ()
                set(CONAN_ARCH  ${CMAKE_HOST_SYSTEM_PROCESSOR})
            endif ()
        elseif (PLATFORM STREQUAL freertos-arm OR PLATFORM STREQUAL baremetal-arm)
            set(CONAN_ARCH      armv7)
        endif ()
    endif ()
    string(REPLACE "armv7l" "armv8_32" CONAN_ARCH ${CONAN_ARCH})

    # Value for settings.arch_build.
    if (CMAKE_CROSSCOMPILING)
        set(CONAN_HOST_ARCH     x86_64)
    else ()
        set(CONAN_HOST_ARCH     ${CONAN_ARCH})
    endif ()

    # Value for settings.compiler.
    if (CMAKE_CXX_COMPILER_ID STREQUAL GNU)
        set(CONAN_COMPILER      gcc)
    else ()
        set(CONAN_COMPILER      clang)
    endif ()

    # Value for settings.compiler.version.
    string(REPLACE "." ";" VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
    list(GET VERSION_LIST 0 CONAN_COMPILER_MAJOR)

    # Values for env.CFLAGS, env.CXXFLAGS and env.LDFLAGS.
    string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
    set(CONAN_C_FLAGS           "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${BUILD_TYPE}}")
    set(CONAN_CXX_FLAGS         "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${BUILD_TYPE}}")
    set(CONAN_LINKER_FLAGS      "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_${BUILD_TYPE}}")

    message(STATUS "Configuring '${CMAKE_BINARY_DIR}/conan-profile'")
    configure_file(${CMAKE_SOURCE_DIR}/tools/profile.in ${CMAKE_BINARY_DIR}/conan-profile)
endfunction ()
