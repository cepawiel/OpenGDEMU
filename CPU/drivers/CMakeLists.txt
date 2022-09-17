project(openGDEMU_Drivers)

add_library(${PROJECT_NAME} 
    src/sd_mmc.c
    src/hsmci.c
    src/smc.c
    src/cyclone_ps.cpp
    src/gpio.cpp
    src/uart.cpp
    )

target_link_libraries(${PROJECT_NAME} 
    asf
    argon
    cmsis
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    # __SAM3U4E__
    # SAM3U
    # BOARD_OSC_STARTUP_US=15625
    # BOARD_MCK=CHIP_FREQ_CPU_MAX

    # # configure chip to run at 96MHz
    # CONFIG_SYSCLK_SOURCE=SYSCLK_SRC_PLLACK
    # CONFIG_SYSCLK_PRES=SYSCLK_PRES_2
    # CONFIG_PLL0_SOURCE=PLL_SRC_MAINCK_XTAL
    # CONFIG_PLL0_MUL=16
    # CONFIG_PLL0_DIV=1

    # BOARD_FREQ_MAINCK_XTAL=12000000UL
)