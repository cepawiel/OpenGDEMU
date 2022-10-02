#
# Copyright (c) 2010 Atmel Corporation. All rights reserved.
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
PRJ_PATH = ../../../../..

# Microcontroller: atxmega128a1, atmega128, attiny261, etc.
MCU = atmega128rfa1

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = examples_avr477_target.elf

# C source files located from the top-level source directory
CSRCS = \
       common/boards/user_board/init.c                    \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target/main.c \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target/qdebug_transport.c \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target/twi_master.c \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target/vendor_data.c

# Assembler source files located from the top-level source directory
ASSRCS = 

# Include path located from the top-level source directory
INC_PATH = \
       common/boards                                      \
       common/boards/user_board                           \
       common/utils                                       \
       mega/utils                                         \
       mega/utils/preprocessor                            \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target \
       thirdparty/atmel_rf4ce/lib                         \
       thirdparty/atmel_rf4ce/lib/inc \
       thirdparty/atmel_rf4ce/examples/atmega128rfa1_avr477_target/gcc

# Library paths from the top-level source directory
LIB_PATH =  \
       thirdparty/atmel_rf4ce/lib/megarf/gcc             

# Libraries to link with the project
LIBS =  \
       _rf4ce_target_no_serial_bootloader                

# Additional options for debugging. By default the common Makefile.in will
# add -gdwarf-2.
DBGFLAGS = 

# Optimization settings
OPTIMIZATION = -Os

# Extra flags used when creating an EEPROM Intel HEX file. By default the
# common Makefile.in will add -j .eeprom
# --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0.
EEPROMFLAGS = 

# Extra flags used when creating an Intel HEX file. By default the common
# Makefile.in will add -R .eeprom -R .usb_descriptor_table.
FLASHFLAGS = 

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS =  \
       -fshort-enums                                     

# Extra flags to use when compiling.
CFLAGS =  \
       -fshort-enums                                     

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
       -D BOARD=USER_BOARD                                \
       -D BOARD_TYPE=RCB_6_3_SENS_TERM_BOARD              \
       -D CHANNEL_AGILITY                                 \
       -D DEBUG=0                                         \
       -D HIGHEST_STACK_LAYER=RF4CE                       \
       -D MAC_USER_BUILD_CONFIG                           \
       -D PAL_GENERIC_TYPE=MEGA_RF_                       \
       -D PAL_TYPE=ATMEGA128RFA1                          \
       -D PB_PAIRING                                      \
       -D QDEBUG_TWI                                      \
       -D RELAY_SUPPORT                                   \
       -D RF4CE_SECURITY                                  \
       -D RF4CE_TARGET                                    \
       -D STB_RCB                                         \
       -D TAL_TYPE=ATMEGARF_TAL_1                         \
       -D VENDOR_DATA                                     \
       -D ZRC_CMD_DISCOVERY                               \
       -D ZRC_PROFILE                                     \
       -D _GENERIC_QDEBUG_

# Extra flags to use when linking
LDFLAGS =  \
       -Wl,--defsym=__eerd_byte_m128rfa1=eeprom_read_byte \
       -Wl,--defsym=__eewr_byte_m128rfa1=eeprom_write_byte \
       -Wl,--section-start=.bootloader=0x1f000            \
       -Wl,-u,eeprom_read_byte                            \
       -Wl,-u,eeprom_write_byte                          

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 