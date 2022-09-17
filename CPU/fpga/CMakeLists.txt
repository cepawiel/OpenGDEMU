project(openGDEMU_FPGA)

add_library(openGDEMU_FPGA
    src/Global.cpp
    src/IDERegs.cpp
    )

target_link_libraries(${PROJECT_NAME} 
    fpga_bitstream
)

target_include_directories(openGDEMU_FPGA
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

# target_compile_definitions(${PROJECT_NAME} PUBLIC
#     # BOARD_OSC_STARTUP_US=15625
# )

