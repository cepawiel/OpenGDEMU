project(SpinalHDL_Container)

# Build SpinalHDL
set(IMAGE_NAME "spinalhdl")
set(SPINALHDL_IMAGE_HASH ${CMAKE_CURRENT_BINARY_DIR}/${IMAGE_NAME}.hash CACHE INTERNAL "")
set(DOCKERFILE ${CMAKE_CURRENT_LIST_DIR}/Dockerfile)
add_custom_command(
    OUTPUT ${SPINALHDL_IMAGE_HASH}
    COMMAND ${Docker_EXECUTABLE} build --tag ${IMAGE_NAME} --file ${DOCKERFILE} ${CMAKE_CURRENT_LIST_DIR}
    COMMAND ${Docker_EXECUTABLE} images --no-trunc ${IMAGE_NAME} > ${SPINALHDL_IMAGE_HASH}
    DEPENDS 
        ${DOCKERFILE}
        ${CMAKE_CURRENT_LIST_DIR}/entrypoint.sh
    USES_TERMINAL
    )

add_custom_target(spinalhdl_container DEPENDS ${SPINALHDL_IMAGE_HASH})