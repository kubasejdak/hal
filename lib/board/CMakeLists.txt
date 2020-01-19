add_library(board-public INTERFACE)
add_library(board-private INTERFACE)

foreach (BOARD IN LISTS HAL_BOARDS)
    add_subdirectory(${BOARD})

    target_link_libraries(board-public
        INTERFACE ${BOARD}-public
    )
    target_link_libraries(board-private
        INTERFACE ${BOARD}-private
    )
endforeach ()
