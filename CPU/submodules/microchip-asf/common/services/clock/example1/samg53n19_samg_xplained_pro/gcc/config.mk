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
PRJ_PATH = ../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m4

# Target part: none, sam3n4 or sam4l4aa
PART = samg53n19

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = clock_example1_flash.elf
TARGET_SRAM = clock_example1_sram.elf

# List of C source files.
CSRCS = \
       common/services/clock/example1/clock_example1_sam.c \
       common/services/clock/samg/sysclk.c                \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       sam/boards/samg53_xplained_pro/board_init.c        \
       sam/drivers/efc/efc.c                              \
       sam/drivers/pmc/pmc.c                              \
       sam/drivers/pmc/sleep.c                            \
       sam/drivers/supc/supc.c                            \
       sam/utils/cmsis/samg/samg53/source/templates/exceptions.c \
       sam/utils/cmsis/samg/samg53/source/templates/gcc/startup_samg53.c \
       sam/utils/cmsis/samg/samg53/source/templates/system_samg53.c \
       sam/utils/syscalls/gcc/syscalls.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/services/clock                              \
       common/services/clock/example1/samg53n19_samg_xplained_pro \
       common/services/ioport                             \
       common/utils                                       \
       sam/boards                                         \
       sam/boards/samg53_xplained_pro                     \
       sam/drivers/efc                                    \
       sam/drivers/pmc                                    \
       sam/drivers/supc                                   \
       sam/utils                                          \
       sam/utils/cmsis/samg/samg53/include                \
       sam/utils/cmsis/samg/samg53/source/templates       \
       sam/utils/fpu                                      \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC \
       common/services/clock/example1/samg53n19_samg_xplained_pro/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM4lf_math_softfp                         \
       m                                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/samg/samg53n19/gcc/flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/samg/samg53n19/gcc/sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/samg53_xplained_pro/debug_scripts/gcc/samg53_xplained_pro_flash.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/samg53_xplained_pro/debug_scripts/gcc/samg53_xplained_pro_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = all

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS =  \
       -mfloat-abi=softfp                                 \
       -mfpu=fpv4-sp-d16                                 

# Extra flags to use when compiling.
CFLAGS =  \
       -mfloat-abi=softfp                                 \
       -mfpu=fpv4-sp-d16                                 

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
       -D ARM_MATH_CM4=true                               \
       -D BOARD=SAMG53_XPLAINED_PRO                       \
       -D __SAMG53N19__                                   \
       -D printf=iprintf                                  \
       -D scanf=iscanf

# Extra flags to use when linking
LDFLAGS = \

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 