add_library(hal-board-interface INTERFACE)
add_library(hal::board-interface ALIAS hal-board-interface)
add_library(hal-board INTERFACE)
add_library(hal::board ALIAS hal-board)
add_library(hal-board-config INTERFACE)
add_library(hal::board-config ALIAS hal-board-config)

target_include_directories(hal-board-interface
    INTERFACE include
)

foreach (BOARD_PATH IN LISTS HAL_BOARDS)
    add_subdirectory(${BOARD_PATH})

    string(REPLACE "/" "-" BOARD "${BOARD_PATH}")
    target_link_libraries(hal-board
        INTERFACE hal::${BOARD}
    )

    if (TARGET hal::${BOARD}-config)
        target_link_libraries(hal-board-config
            INTERFACE hal::${BOARD}-config
        )
    endif ()
endforeach ()
