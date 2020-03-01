if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake" "${CMAKE_BINARY_DIR}/conan.cmake")
endif ()

include(${CMAKE_BINARY_DIR}/conan.cmake)

macro(conan_get)
    set(CMAKE_SYSTEM_NAME_TMP   ${CMAKE_SYSTEM_NAME})
    set(CMAKE_SYSTEM_NAME       ${CMAKE_HOST_SYSTEM_NAME})
    conan_cmake_run(
        REQUIRES                ${ARGN}
        PROFILE                 ${CONAN_PROFILE}
        PROFILE_AUTO            build_type
        BUILD                   missing
        GENERATORS              cmake_find_package cmake_paths
    )
    set(CMAKE_SYSTEM_NAME       ${CMAKE_SYSTEM_NAME_TMP})

    include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake)
endmacro ()
