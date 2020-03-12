include(${CMAKE_CURRENT_LIST_DIR}/conan-wrapper.cmake)

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
