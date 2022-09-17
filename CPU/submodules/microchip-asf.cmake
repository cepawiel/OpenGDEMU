project(asf)

add_library(${PROJECT_NAME} 
    # ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils/cmsis/sam3u/source/templates/exceptions.c

    # PIO Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pio/pio_handler.c
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pio/pio.c

    # PMC Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pmc/pmc.c
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pmc/sleep.c

    # Watchdog Driver
    # ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/hsmci/hsmci.c

    # HSMCI Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/wdt/wdt.c

    # FATFS Port
    # ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/fatfs/fatfs-port-r0.09/diskio.c
    # ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/fatfs/fatfs-port-r0.09/sam/fattime_rtc.c
    # ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/fatfs/fatfs-r0.09/src/ff.c

    # DMAC Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/dmac/dmac.c

    # RTC Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/rtc/rtc.c

    # PWM Driver
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pwm/pwm.c

    # SYSCLOCK
    ${PROJECT_SOURCE_DIR}/microchip-asf/common/services/clock/sam3u/sysclk.c

    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/efc/efc.c
)

target_include_directories(${PROJECT_NAME}
    # PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils/cmsis/sam3u/include
    # PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils/cmsis/sam3u/source/templates
    # PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/CMSIS/Include
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils/preprocessor
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/utils/header_files
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/common/utils
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/common/boards

    # Drivers
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/pmc/
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/dmac/
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/rtc/
    ${PROJECT_SOURCE_DIR}/microchip-asf/sam/drivers/efc/

    # sd_mmc
    # ${PROJECT_SOURCE_DIR}/microchip-asf/common/components/memory/sd_mmc/
    ${PROJECT_SOURCE_DIR}/microchip-asf/common/services/clock/

    # FATFS
    # ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/fatfs/fatfs-r0.09/src/
    # ${PROJECT_SOURCE_DIR}/microchip-asf/common/services/storage/ctrl_access/
    

    # sysclk
    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/common/services/clock/sam3u/

    PUBLIC ${PROJECT_SOURCE_DIR}/microchip-asf/thirdparty/CMSIS/Include/
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    __SAM3U4E__
    BOARD_OSC_STARTUP_US=15625
    BOARD_MCK=CHIP_FREQ_CPU_MAX

    # configure chip to run at 96MHz
    CONFIG_SYSCLK_SOURCE=SYSCLK_SRC_PLLACK
    CONFIG_SYSCLK_PRES=SYSCLK_PRES_2
    CONFIG_PLL0_SOURCE=PLL_SRC_MAINCK_XTAL
    CONFIG_PLL0_MUL=16
    CONFIG_PLL0_DIV=1

    BOARD_FREQ_MAINCK_XTAL=12000000UL
)

target_link_options(${PROJECT_NAME} PUBLIC 
    # "-nostdlib"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    # define to skip board.h header file
    BOARD=STK600_MEGA
)

target_link_libraries(${PROJECT_NAME} 
    cmsis
)
