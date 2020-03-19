include(${CMAKE_CURRENT_LIST_DIR}/conan-wrapper.cmake)

function(conan_init)
    # Value for settings.os.
    if (PLATFORM STREQUAL linux)
        if (CMAKE_HOST_SYSTEM_NAME STREQUAL Linux)
            set(CONAN_OS        Linux)
        else ()
            set(CONAN_OS        Macos)
        endif ()
    elseif (PLATFORM STREQUAL linux-arm)
        set(CONAN_OS            Linux)
    elseif (PLATFORM STREQUAL freertos-arm OR PLATFORM STREQUAL baremetal-arm)
        set(CONAN_OS            Generic)
    endif ()

    # Value for settings.os_build.
    if (CMAKE_HOST_SYSTEM_NAME STREQUAL Linux)
        set(CONAN_HOST_OS       Linux)
    else ()
        set(CONAN_HOST_OS       Macos)
    endif ()

    # Value for settings.arch.
    if (NOT DEFINED CONAN_ARCH)
        if (PLATFORM STREQUAL linux)
            if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES arm)
                set(CONAN_ARCH  armv8_32)
            else ()
                set(CONAN_ARCH  x86_64)
            endif ()
        elseif (PLATFORM STREQUAL linux-arm)
            set(CONAN_ARCH      armv8_32)
        elseif (PLATFORM STREQUAL freertos-arm OR PLATFORM STREQUAL baremetal-arm)
            set(CONAN_ARCH      armv7)
        endif ()
    endif ()

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

    # Value for settings.compiler.libcxx.
    if (CONAN_OS STREQUAL Macos AND CONAN_COMPILER STREQUAL clang)
        set(CONAN_LIBCXX        libc++)
    else ()
        set(CONAN_LIBCXX        libstdc++11)
    endif ()

    # Values for env.CFLAGS, env.CXXFLAGS and env.LDFLAGS.
    string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
    set(CONAN_C_FLAGS           "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${BUILD_TYPE}}")
    set(CONAN_CXX_FLAGS         "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${BUILD_TYPE}}")
    set(CONAN_LINKER_FLAGS      "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_${BUILD_TYPE}}")

    message(STATUS "Configuring '${CMAKE_BINARY_DIR}/conan-profile'")
    configure_file(${CMAKE_SOURCE_DIR}/tools/profile.in ${CMAKE_BINARY_DIR}/conan-profile)
endfunction ()

macro(conan_get)
    set(CMAKE_SYSTEM_NAME_TMP   ${CMAKE_SYSTEM_NAME})
    set(CMAKE_SYSTEM_NAME       ${CMAKE_HOST_SYSTEM_NAME})
    conan_cmake_run(
        REQUIRES                ${ARGN}
        PROFILE                 ${CMAKE_BINARY_DIR}/conan-profile
        BUILD                   missing
        GENERATORS              cmake_find_package cmake_paths
    )
    set(CMAKE_SYSTEM_NAME       ${CMAKE_SYSTEM_NAME_TMP})

    include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake)
endmacro ()
