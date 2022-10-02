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
PART = samr21g18a

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = apps_wsn_demo_flash.elf
TARGET_SRAM = apps_wsn_demo_sram.elf

# List of C source files.
CSRCS = \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common2/services/delay/sam0/systick_counter.c      \
       sam0/boards/samr21zll_ek/board_init.c              \
       sam0/drivers/extint/extint_callback.c              \
       sam0/drivers/extint/extint_sam_d_r_h/extint.c      \
       sam0/drivers/nvm/nvm.c                             \
       sam0/drivers/port/port.c                           \
       sam0/drivers/rtc/rtc_sam_d_r_h/rtc_count.c         \
       sam0/drivers/rtc/rtc_sam_d_r_h/rtc_count_interrupt.c \
       sam0/drivers/sercom/sercom.c                       \
       sam0/drivers/sercom/sercom_interrupt.c             \
       sam0/drivers/sercom/spi/spi.c                      \
       sam0/drivers/sercom/usart/usart.c                  \
       sam0/drivers/sercom/usart/usart_interrupt.c        \
       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/clock.c \
       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/gclk.c \
       sam0/drivers/system/interrupt/system_interrupt.c   \
       sam0/drivers/system/pinmux/pinmux.c                \
       sam0/drivers/system/system.c                       \
       sam0/drivers/tc/tc_interrupt.c                     \
       sam0/drivers/tc/tc_sam_d_r_h/tc.c                  \
       sam0/utils/cmsis/samr21/source/gcc/startup_samr21.c \
       sam0/utils/cmsis/samr21/source/system_samr21.c     \
       sam0/utils/stdio/read.c                            \
       sam0/utils/stdio/write.c                           \
       sam0/utils/syscalls/gcc/syscalls.c                 \
       thirdparty/wireless/addons/sio2host/uart/sio2host.c \
       thirdparty/wireless/miwi/apps/wsn_demo/commands.c  \
       thirdparty/wireless/miwi/apps/wsn_demo/main.c      \
       thirdparty/wireless/miwi/apps/wsn_demo/wsndemo.c   \
       thirdparty/wireless/miwi/services/otau/circularBuffer.c \
       thirdparty/wireless/miwi/services/otau/debug/client_debug.c \
       thirdparty/wireless/miwi/services/otau/debug/server_debug.c \
       thirdparty/wireless/miwi/services/otau/notify/client_notify.c \
       thirdparty/wireless/miwi/services/otau/notify/server_notify.c \
       thirdparty/wireless/miwi/services/otau/otau.c      \
       thirdparty/wireless/miwi/services/otau/otau_parser.c \
       thirdparty/wireless/miwi/services/otau/upgrade/client_upgrade.c \
       thirdparty/wireless/miwi/services/otau/upgrade/server_upgrade.c \
       thirdparty/wireless/miwi/services/pds/src/nopds/fakePds.c \
       thirdparty/wireless/miwi/services/pds/src/nv/D_Nv.c \
       thirdparty/wireless/miwi/services/pds/src/nv/External/D_XNv-SamR21.c \
       thirdparty/wireless/miwi/services/pds/src/nv/External/S_Nv-External.c \
       thirdparty/wireless/miwi/services/pds/src/nv/External/S_XNv.c \
       thirdparty/wireless/miwi/services/pds/src/nv/S_Nv-SamR21.c \
       thirdparty/wireless/miwi/services/pds/src/wl/wlPdsDataServer.c \
       thirdparty/wireless/miwi/services/pds/src/wl/wlPdsInit.c \
       thirdparty/wireless/miwi/services/pds/src/wl/wlPdsTaskManager.c \
       thirdparty/wireless/miwi/services/pds/src/wl/wlPdsTypesConverter.c \
       thirdparty/wireless/miwi/services/sleep_mgr/sam0/sleep_mgr.c \
       thirdparty/wireless/miwi/source/mimac/mimac_at86rf.c \
       thirdparty/wireless/miwi/source/mimac/phy/at86rf233/phy.c \
       thirdparty/wireless/miwi/source/miwi_mesh/miwi_mesh_app.c \
       thirdparty/wireless/miwi/source/miwi_mesh/miwi_mesh_pds.c \
       thirdparty/wireless/miwi/source/stb/src/stb.c      \
       thirdparty/wireless/miwi/source/stb/src/stb_armcrypto.c \
       thirdparty/wireless/miwi/source/stb/src/stb_help.c \
       thirdparty/wireless/miwi/source/sys/mimem.c        \
       thirdparty/wireless/miwi/source/sys/miqueue.c      \
       thirdparty/wireless/miwi/source/sys/sysTimer.c     \
       thirdparty/wireless/services/common_hw_timer/sam0/hw_timer.c \
       thirdparty/wireless/services/nvm/sam0/sam_nvm.c    \
       thirdparty/wireless/services/sal/at86rf2xx/src/sal.c \
       thirdparty/wireless/services/trx_access/trx_access.c

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
       sam0/boards/samr21zll_ek                           \
       sam0/drivers/extint                                \
       sam0/drivers/extint/extint_sam_d_r_h               \
       sam0/drivers/nvm                                   \
       sam0/drivers/port                                  \
       sam0/drivers/rtc                                   \
       sam0/drivers/rtc/rtc_sam_d_r_h                     \
       sam0/drivers/sercom                                \
       sam0/drivers/sercom/spi                            \
       sam0/drivers/sercom/usart                          \
       sam0/drivers/system                                \
       sam0/drivers/system/clock                          \
       sam0/drivers/system/clock/clock_samd21_r21_da_ha1  \
       sam0/drivers/system/interrupt                      \
       sam0/drivers/system/interrupt/system_interrupt_samr21 \
       sam0/drivers/system/pinmux                         \
       sam0/drivers/system/power                          \
       sam0/drivers/system/power/power_sam_d_r_h          \
       sam0/drivers/system/reset                          \
       sam0/drivers/system/reset/reset_sam_d_r_h          \
       sam0/drivers/tc                                    \
       sam0/drivers/tc/tc_sam_d_r_h                       \
       sam0/utils                                         \
       sam0/utils/cmsis/samr21/include                    \
       sam0/utils/cmsis/samr21/source                     \
       sam0/utils/header_files                            \
       sam0/utils/preprocessor                            \
       sam0/utils/stdio/stdio_serial                      \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/wireless/addons/sio2host/uart           \
       thirdparty/wireless/miwi/apps/wsn_demo             \
       thirdparty/wireless/miwi/apps/wsn_demo/ed_sec/samr21_ek \
       thirdparty/wireless/miwi/include                   \
       thirdparty/wireless/miwi/services/otau             \
       thirdparty/wireless/miwi/services/otau/debug       \
       thirdparty/wireless/miwi/services/otau/notify      \
       thirdparty/wireless/miwi/services/otau/upgrade     \
       thirdparty/wireless/miwi/services/pds/inc          \
       thirdparty/wireless/miwi/services/pds/inc/nv       \
       thirdparty/wireless/miwi/services/pds/inc/nv/External \
       thirdparty/wireless/miwi/services/pds/inc/wl       \
       thirdparty/wireless/miwi/services/sleep_mgr        \
       thirdparty/wireless/miwi/source/mimac              \
       thirdparty/wireless/miwi/source/mimac/phy          \
       thirdparty/wireless/miwi/source/miwi_mesh          \
       thirdparty/wireless/miwi/source/stb/inc            \
       thirdparty/wireless/miwi/source/sys                \
       thirdparty/wireless/services/common_hw_timer       \
       thirdparty/wireless/services/common_hw_timer/sam0  \
       thirdparty/wireless/services/nvm                   \
       thirdparty/wireless/services/sal/inc               \
       thirdparty/wireless/services/trx_access \
       thirdparty/wireless/miwi/apps/wsn_demo/ed_sec/samr21_ek/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/wireless/miwi/source/miwi_mesh/sam0/gcc \
       thirdparty/wireless/services/common_hw_timer/sam0/lib

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM0l_math                                 \
       sam0_miwi_mesh_ed_sec                              \
       sam0_lib_hw_timer                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = thirdparty/wireless/miwi/services/pds/src/wl/linkerscripts/samr21/gcc/samr21g18a_flash.ld
LINKER_SCRIPT_SRAM  = thirdparty/wireless/miwi/services/pds/src/wl/linkerscripts/samr21/gcc/samr21g18a_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam0/boards/samr21zll_ek/debug_scripts/gcc/samr21zll_ek_flash.gdb
DEBUG_SCRIPT_SRAM  = sam0/boards/samr21zll_ek/debug_scripts/gcc/samr21zll_ek_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O3

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
       -D ARM_MATH_CM0PLUS=true                           \
       -D BOARD=SAMR21ZLL_EK                              \
       -D ENABLE_NETWORK_FREEZER                          \
       -D ENABLE_SLEEP_FEATURE                            \
       -D ENDDEVICE                                       \
       -D EXTINT_CALLBACK_MODE=true                       \
       -D MESH_SECURITY                                   \
       -D OTAU_ENABLED                                    \
       -D PDS_ENABLE_WEAR_LEVELING                        \
       -D PHY_AT86RF233                                   \
       -D PROTOCOL_MESH                                   \
       -D RTC_COUNT_ASYNC=true                            \
       -D SAL_TYPE=AT86RF2xx                              \
       -D SPI_CALLBACK_MODE=false                         \
       -D STB_ON_SAL                                      \
       -D SYSTICK_MODE                                    \
       -D TC_ASYNC=true                                   \
       -D USART_CALLBACK_MODE=true                        \
       -D __SAMR21G18A__

# Extra flags to use when linking
LDFLAGS = \

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 