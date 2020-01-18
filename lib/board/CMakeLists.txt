add_library(board_public INTERFACE)
add_library(board_private INTERFACE)

foreach (BOARD IN LISTS BOARDS)
    add_subdirectory(${BOARD})

    target_link_libraries(board_public
        INTERFACE ${BOARD}_public
    )
    target_link_libraries(board_private
        INTERFACE ${BOARD}_private
    )
endforeach ()
