project(mainapp)

set(LINKER_SCRIPT_PATH "${PROJECT_SOURCE_DIR}/sam3u4e_flash.ld")

add_executable(${PROJECT_NAME} 
    main.cpp
    cyclone2.cpp

    pata/PATADevice.cpp
    pata/HDD_LBA48.cpp

    )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -save-temps")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -save-temps")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-T${LINKER_SCRIPT_PATH} -Wl,-Map=${PROJECT_NAME}.map")

target_link_libraries(${PROJECT_NAME}   
    openGDEMU_FPGA
    openGDEMU_Drivers
    asf
    argon
    log
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.elf")

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}
)

add_custom_target(${PROJECT_NAME}.bin ALL DEPENDS ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME}.bin
COMMAND ${CMAKE_OBJCOPY} ARGS -O binary "${PROJECT_NAME}.elf" ${PROJECT_NAME}.bin)
