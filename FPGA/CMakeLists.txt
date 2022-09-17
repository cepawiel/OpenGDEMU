project(FPGA)
add_subdirectory(spinalhdl)
add_subdirectory(quartus)

file(GLOB_RECURSE verilog_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/sources/**")

set(SYMBIYOSYS_VOLUMES -v ${CMAKE_CURRENT_LIST_DIR}/sources:/input:ro -v ${CMAKE_CURRENT_BINARY_DIR}:/output)
set(SYMBIYOSYS_OUTPUTS "${SYMBIYOSYS_OUTPUTS} ${sby_file}.test")
set(sby_file demo)
add_custom_command(
    OUTPUT ${sby_file}.test
    COMMAND ${Docker_EXECUTABLE} run --rm ${SYMBIYOSYS_VOLUMES} symbiyosys sby -f ${sby_file}.sby
    DEPENDS 
        ${SYMBIYOSYS_IMAGE_HASH}
        ${GENERATED_VERILOG_OUTPUT}
        ${verilog_SOURCES}
    )
add_custom_target(${sby_file}_formal_verification DEPENDS ${sby_file}.test)



# FPGA Bitstream
add_custom_command(
    OUTPUT bitstream.o
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMAND
        cd ${QUARTUS_OUTPUT_RBF_FOLDER} && 
        arm-none-eabi-objcopy -I binary -O elf32-littlearm --rename-section .data=.rodata,alloc,load,readonly,data,contents OpenGDEMU.rbf ${CMAKE_CURRENT_BINARY_DIR}/bitstream.o
    DEPENDS 
        ${QUARTUS_OUTPUT_RBF}
        ${sby_file}.test
    )

SET_SOURCE_FILES_PROPERTIES(
    bitstream.o
    PROPERTIES
    EXTERNAL_OBJECT true
    GENERATED true
    )

add_library(fpga_bitstream bitstream.o)
set_target_properties(fpga_bitstream PROPERTIES LINKER_LANGUAGE C)
