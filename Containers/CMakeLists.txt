project(Containers)

execute_process(
    COMMAND whoami
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE USER_NAME
    )

execute_process(
    COMMAND id -u ${USER_NAME}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE USER_UID
    )

execute_process(
    COMMAND id -g ${USER_NAME}
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE USER_GID
    )

message(STATUS "Building as ${USER_NAME}")
message(STATUS "UID: ${USER_UID}")
message(STATUS "GID: ${USER_GID}")

add_subdirectory(QuartusII)
add_subdirectory(SpinalHDL)
add_subdirectory(SymbiYosys)