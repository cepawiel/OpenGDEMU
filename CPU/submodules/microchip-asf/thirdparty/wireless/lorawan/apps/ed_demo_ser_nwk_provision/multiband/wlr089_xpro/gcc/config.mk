#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ../../../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m0plus

# Target part: none, sam3n4 or sam4l4aa
PART = wlr089u0

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = apps_ed_demo_ser_nwk_provision_flash.elf
TARGET_SRAM = apps_ed_demo_ser_nwk_provision_sram.elf

# List of C source files.
CSRCS = \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common2/services/delay/sam0/systick_counter.c      \
       sam0/boards/wlr089_xplained_pro/board_init.c       \
       sam0/drivers/adc/adc_sam_l_c/adc.c                 \
       sam0/drivers/adc/adc_sam_l_c/adc_callback.c        \
       sam0/drivers/aes/aes.c                             \
       sam0/drivers/extint/extint_callback.c              \
       sam0/drivers/extint/extint_sam_l_c/extint.c        \
       sam0/drivers/nvm/nvm.c                             \
       sam0/drivers/port/port.c                           \
       sam0/drivers/rtc/rtc_sam_l_c/rtc_count.c           \
       sam0/drivers/rtc/rtc_sam_l_c/rtc_count_interrupt.c \
       sam0/drivers/sercom/i2c/i2c_sam0/i2c_master.c      \
       sam0/drivers/sercom/i2c/i2c_sam0/i2c_master_interrupt.c \
       sam0/drivers/sercom/sercom.c                       \
       sam0/drivers/sercom/sercom_interrupt.c             \
       sam0/drivers/sercom/spi/spi.c                      \
       sam0/drivers/sercom/usart/usart.c                  \
       sam0/drivers/sercom/usart/usart_interrupt.c        \
       sam0/drivers/system/clock/clock_wlr089/clock.c     \
       sam0/drivers/system/clock/clock_wlr089/gclk.c      \
       sam0/drivers/system/interrupt/system_interrupt.c   \
       sam0/drivers/system/pinmux/pinmux.c                \
       sam0/drivers/system/system.c                       \
       sam0/drivers/tc/tc_interrupt.c                     \
       sam0/drivers/tc/tc_sam_l_c/tc.c                    \
       sam0/utils/cmsis/wlr089/source/gcc/startup_wlr089.c \
       sam0/utils/cmsis/wlr089/source/system_wlr089.c     \
       sam0/utils/stdio/read.c                            \
       sam0/utils/stdio/write.c                           \
       sam0/utils/syscalls/gcc/syscalls.c                 \
       thirdparty/wireless/addons/sio2host/uart/sio2host.c \
       thirdparty/wireless/lorawan/apps/ed_demo_ser_nwk_provision/enddevice_demo.c \
       thirdparty/wireless/lorawan/apps/ed_demo_ser_nwk_provision/main.c \
       thirdparty/wireless/lorawan/hal/src/radio_driver_hal.c \
       thirdparty/wireless/lorawan/hal/src/sleep/sam0/sleep.c \
       thirdparty/wireless/lorawan/hal/src/sleep_timer/sam0/sleep_timer.c \
       thirdparty/wireless/lorawan/hal/src/sys.c          \
       thirdparty/wireless/lorawan/pmm/src/pmm.c          \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_as.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_au.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_eu.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_in.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_jp.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_kr.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_mband_na.c \
       thirdparty/wireless/lorawan/regparams/multiband/src/lorawan_multiband.c \
       thirdparty/wireless/lorawan/sal/src/sal.c          \
       thirdparty/wireless/lorawan/services/aes/src/hw/sam0/aes_engine.c \
       thirdparty/wireless/lorawan/services/edbg_eui/edbg_eui.c \
       thirdparty/wireless/lorawan/services/pds/src/pds_interface.c \
       thirdparty/wireless/lorawan/services/pds/src/pds_nvm.c \
       thirdparty/wireless/lorawan/services/pds/src/pds_task_handler.c \
       thirdparty/wireless/lorawan/services/pds/src/pds_wl.c \
       thirdparty/wireless/lorawan/services/resources/src/LED.c \
       thirdparty/wireless/lorawan/services/resources/src/resources.c \
       thirdparty/wireless/lorawan/services/resources/src/temp_sensor.c \
       thirdparty/wireless/lorawan/services/sw_timer/src/sw_timer.c \
       thirdparty/wireless/lorawan/sys/src/system_assert.c \
       thirdparty/wireless/lorawan/sys/src/system_init.c  \
       thirdparty/wireless/lorawan/sys/src/system_low_power.c \
       thirdparty/wireless/lorawan/sys/src/system_task_manager.c \
       thirdparty/wireless/lorawan/tal/sx1276/src/radio_driver_SX1276.c \
       thirdparty/wireless/services/common_hw_timer/sam0/hw_timer.c \
       thirdparty/wireless/services/nvm/sam0/sam_nvm.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/serial                             \
       common/utils                                       \
       common2/services/delay                             \
       common2/services/delay/sam0                        \
       sam0/boards                                        \
       sam0/boards/wlr089_xplained_pro                    \
       sam0/drivers/adc                                   \
       sam0/drivers/adc/adc_sam_l_c                       \
       sam0/drivers/aes                                   \
       sam0/drivers/extint                                \
       sam0/drivers/extint/extint_sam_l_c                 \
       sam0/drivers/nvm                                   \
       sam0/drivers/port                                  \
       sam0/drivers/rtc                                   \
       sam0/drivers/rtc/rtc_sam_l_c                       \
       sam0/drivers/sercom                                \
       sam0/drivers/sercom/i2c                            \
       sam0/drivers/sercom/i2c/i2c_sam0                   \
       sam0/drivers/sercom/spi                            \
       sam0/drivers/sercom/usart                          \
       sam0/drivers/system                                \
       sam0/drivers/system/clock                          \
       sam0/drivers/system/clock/clock_wlr089             \
       sam0/drivers/system/interrupt                      \
       sam0/drivers/system/interrupt/system_interrupt_wlr089 \
       sam0/drivers/system/pinmux                         \
       sam0/drivers/system/power                          \
       sam0/drivers/system/power/power_sam_l              \
       sam0/drivers/system/reset                          \
       sam0/drivers/system/reset/reset_sam_l              \
       sam0/drivers/tc                                    \
       sam0/drivers/tc/tc_sam_l_c                         \
       sam0/utils                                         \
       sam0/utils/cmsis/wlr089/include                    \
       sam0/utils/cmsis/wlr089/source                     \
       sam0/utils/header_files                            \
       sam0/utils/preprocessor                            \
       sam0/utils/stdio/stdio_serial                      \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/wireless/addons/sio2host/uart           \
       thirdparty/wireless/lorawan/apps/ed_demo_ser_nwk_provision \
       thirdparty/wireless/lorawan/apps/ed_demo_ser_nwk_provision/multiband/wlr089_xpro \
       thirdparty/wireless/lorawan/hal/inc                \
       thirdparty/wireless/lorawan/inc                    \
       thirdparty/wireless/lorawan/mac/inc                \
       thirdparty/wireless/lorawan/pmm/inc                \
       thirdparty/wireless/lorawan/regparams/inc          \
       thirdparty/wireless/lorawan/regparams/multiband/inc \
       thirdparty/wireless/lorawan/sal/inc                \
       thirdparty/wireless/lorawan/services/aes/inc       \
       thirdparty/wireless/lorawan/services/edbg_eui      \
       thirdparty/wireless/lorawan/services/pds/inc       \
       thirdparty/wireless/lorawan/services/resources/inc \
       thirdparty/wireless/lorawan/services/sw_timer/inc  \
       thirdparty/wireless/lorawan/sys/inc                \
       thirdparty/wireless/lorawan/tal/inc                \
       thirdparty/wireless/lorawan/tal/sx1276/inc         \
       thirdparty/wireless/services/common_hw_timer       \
       thirdparty/wireless/services/common_hw_timer/sam0  \
       thirdparty/wireless/services/nvm \
       thirdparty/wireless/lorawan/apps/ed_demo_ser_nwk_provision/multiband/wlr089_xpro/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/wireless/lorawan/libgen/sam0/gcc        \
       thirdparty/wireless/services/common_hw_timer/sam0/lib

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM0l_math                                 \
       LORAWAN_LIBGEN                                     \
       sam0_lib_hw_timer                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam0/utils/linker_scripts/wlr089/gcc/wlr089u0_flash.ld
LINKER_SCRIPT_SRAM  = sam0/utils/linker_scripts/wlr089/gcc/wlr089u0_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam0/boards/wlr089_xplained_pro/debug_scripts/gcc/wlr089_xplained_pro_flash.gdb
DEBUG_SCRIPT_SRAM  = sam0/boards/wlr089_xplained_pro/debug_scripts/gcc/wlr089_xplained_pro_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -Os

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ADC_CALLBACK_MODE=true                          \
       -D ARM_MATH_CM0PLUS=true                           \
       -D AS_BAND=1                                       \
       -D AU_BAND=1                                       \
       -D BOARD=WLR089_XPLAINED_PRO                       \
       -D CONF_PMM_ENABLE                                 \
       -D EDBG_EUI_READ=0                                 \
       -D ENABLE_PDS=1                                    \
       -D EU_BAND=1                                       \
       -D EXTINT_CALLBACK_MODE=true                       \
       -D I2C_MASTER_CALLBACK_MODE=true                   \
       -D IND_BAND=1                                      \
       -D JPN_BAND=1                                      \
       -D KR_BAND=1                                       \
       -D MODULE_EUI_READ=0                               \
       -D NA_BAND=1                                       \
       -D RANDOM_NW_ACQ=0                                 \
       -D RTC_COUNT_ASYNC=true                            \
       -D SPI_CALLBACK_MODE=false                         \
       -D SYSTICK_MODE                                    \
       -D TC_ASYNC=true                                   \
       -D USART_CALLBACK_MODE=true                        \
       -D _DEBUG_=0                                       \
       -D __WLR089U0__

# Extra flags to use when linking
LDFLAGS = \

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 