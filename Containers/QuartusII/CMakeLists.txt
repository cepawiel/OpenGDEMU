project(QuartusII_Container)

# Build QuartusII 13.0
set(IMAGE_NAME "quartus13")
set(QUARTUS_IMAGE_HASH ${CMAKE_CURRENT_BINARY_DIR}/${IMAGE_NAME}.hash CACHE INTERNAL "")
set(DOCKERFILE ${CMAKE_CURRENT_LIST_DIR}/Dockerfile)
add_custom_command(
    OUTPUT ${QUARTUS_IMAGE_HASH}
    COMMAND ${Docker_EXECUTABLE} build --tag ${IMAGE_NAME} --file ${DOCKERFILE} ${CMAKE_CURRENT_LIST_DIR}
    COMMAND ${Docker_EXECUTABLE} images --no-trunc ${IMAGE_NAME} > ${QUARTUS_IMAGE_HASH}
    DEPENDS 
        ${DOCKERFILE}
        ${CMAKE_CURRENT_LIST_DIR}/entrypoint.sh
    USES_TERMINAL
    )

add_custom_target(quartusII_container DEPENDS ${QUARTUS_IMAGE_HASH})