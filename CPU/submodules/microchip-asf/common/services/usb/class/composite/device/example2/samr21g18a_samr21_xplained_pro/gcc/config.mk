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
PRJ_PATH = ../../../../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m0plus

# Target part: none, sam3n4 or sam4l4aa
PART = samr21g18a

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = example_hids_cdc_msc_flash.elf
TARGET_SRAM = example_hids_cdc_msc_sram.elf

# List of C source files.
CSRCS = \
       common/components/memory/virtual_mem/virtual_mem.c \
       common/services/sleepmgr/samd/sleepmgr.c           \
       common/services/storage/ctrl_access/ctrl_access.c  \
       common/services/usb/class/cdc/device/udi_cdc.c     \
       common/services/usb/class/composite/device/example2/main.c \
       common/services/usb/class/composite/device/example2/memories_initialization_samd.c \
       common/services/usb/class/composite/device/example2/samr21g18a_samr21_xplained_pro/ui.c \
       common/services/usb/class/composite/device/example2/uart_samd.c \
       common/services/usb/class/composite/device/udi_composite_desc.c \
       common/services/usb/class/hid/device/kbd/udi_hid_kbd.c \
       common/services/usb/class/hid/device/mouse/udi_hid_mouse.c \
       common/services/usb/class/hid/device/udi_hid.c     \
       common/services/usb/class/msc/device/udi_msc.c     \
       common/services/usb/udc/udc.c                      \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       sam0/boards/samr21_xplained_pro/board_init.c       \
       sam0/drivers/extint/extint_callback.c              \
       sam0/drivers/extint/extint_sam_d_r_h/extint.c      \
       sam0/drivers/port/port.c                           \
       sam0/drivers/sercom/sercom.c                       \
       sam0/drivers/sercom/sercom_interrupt.c             \
       sam0/drivers/sercom/usart/usart.c                  \
       sam0/drivers/sercom/usart/usart_interrupt.c        \
       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/clock.c \
       sam0/drivers/system/clock/clock_samd21_r21_da_ha1/gclk.c \
       sam0/drivers/system/interrupt/system_interrupt.c   \
       sam0/drivers/system/pinmux/pinmux.c                \
       sam0/drivers/system/system.c                       \
       sam0/drivers/usb/stack_interface/usb_device_udd.c  \
       sam0/drivers/usb/stack_interface/usb_dual.c        \
       sam0/drivers/usb/usb_sam_d_r/usb.c                 \
       sam0/utils/cmsis/samr21/source/gcc/startup_samr21.c \
       sam0/utils/cmsis/samr21/source/system_samr21.c     \
       sam0/utils/syscalls/gcc/syscalls.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/components/memory/virtual_mem               \
       common/services/sleepmgr                           \
       common/services/storage/ctrl_access                \
       common/services/usb                                \
       common/services/usb/class/cdc                      \
       common/services/usb/class/cdc/device               \
       common/services/usb/class/composite/device         \
       common/services/usb/class/composite/device/example2 \
       common/services/usb/class/composite/device/example2/samr21g18a_samr21_xplained_pro \
       common/services/usb/class/hid                      \
       common/services/usb/class/hid/device               \
       common/services/usb/class/hid/device/kbd           \
       common/services/usb/class/hid/device/mouse         \
       common/services/usb/class/msc                      \
       common/services/usb/class/msc/device               \
       common/services/usb/udc                            \
       common/utils                                       \
       sam0/boards                                        \
       sam0/boards/samr21_xplained_pro                    \
       sam0/drivers/extint                                \
       sam0/drivers/extint/extint_sam_d_r_h               \
       sam0/drivers/port                                  \
       sam0/drivers/sercom                                \
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
       sam0/drivers/usb                                   \
       sam0/drivers/usb/stack_interface                   \
       sam0/drivers/usb/usb_sam_d_r                       \
       sam0/utils                                         \
       sam0/utils/cmsis/samr21/include                    \
       sam0/utils/cmsis/samr21/source                     \
       sam0/utils/header_files                            \
       sam0/utils/preprocessor                            \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC \
       common/services/usb/class/composite/device/example2/samr21g18a_samr21_xplained_pro/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM0l_math                                

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam0/utils/linker_scripts/samr21/gcc/samr21g18a_flash.ld
LINKER_SCRIPT_SRAM  = sam0/utils/linker_scripts/samr21/gcc/samr21g18a_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam0/boards/samr21_xplained_pro/debug_scripts/gcc/samr21_xplained_pro_flash.gdb
DEBUG_SCRIPT_SRAM  = sam0/boards/samr21_xplained_pro/debug_scripts/gcc/samr21_xplained_pro_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

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
       -D ACCESS_USB_ENABLED                              \
       -D ARM_MATH_CM0PLUS=true                           \
       -D BOARD=SAMR21_XPLAINED_PRO                       \
       -D EXTINT_CALLBACK_MODE=true                       \
       -D UDD_ENABLE                                      \
       -D USART_CALLBACK_MODE=true                        \
       -D USB_DEVICE_LPM_SUPPORT                          \
       -D VIRTUAL_MEMORY_ENABLE                           \
       -D __SAMR21G18A__

# Extra flags to use when linking
LDFLAGS = \
                                                          \
       -Wl,--defsym,STACK_SIZE=0x0B00                     \
       -Wl,--defsym,__stack_size__=0x0B00

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 