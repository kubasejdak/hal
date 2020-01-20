add_library(board-public INTERFACE)
add_library(board-private INTERFACE)

foreach (BOARD_PATH IN LISTS HAL_BOARDS)
    add_subdirectory(${BOARD_PATH})

    get_filename_component(BOARD ${BOARD_PATH} NAME)
    target_link_libraries(board-public
        INTERFACE ${BOARD}-public
    )
    target_link_libraries(board-private
        INTERFACE ${BOARD}-private
    )
endforeach ()
