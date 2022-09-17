project(SymbiYosys_Container)

# Build SymbiYosys
set(IMAGE_NAME "symbiyosys")
set(SYMBIYOSYS_IMAGE_HASH ${CMAKE_CURRENT_BINARY_DIR}/${IMAGE_NAME}.hash CACHE INTERNAL "")
set(DOCKERFILE ${CMAKE_CURRENT_LIST_DIR}/Dockerfile)
add_custom_command(
    OUTPUT ${SYMBIYOSYS_IMAGE_HASH}
    COMMAND ${Docker_EXECUTABLE} build --tag ${IMAGE_NAME} --file ${DOCKERFILE} ${CMAKE_CURRENT_LIST_DIR}
    COMMAND ${Docker_EXECUTABLE} images --no-trunc ${IMAGE_NAME} > ${SYMBIYOSYS_IMAGE_HASH}
    DEPENDS 
        ${DOCKERFILE}
        # ${CMAKE_CURRENT_LIST_DIR}/run_formal.sh
    USES_TERMINAL
    )

add_custom_target(symbiyosys_image DEPENDS ${SYMBIYOSYS_IMAGE_HASH})
