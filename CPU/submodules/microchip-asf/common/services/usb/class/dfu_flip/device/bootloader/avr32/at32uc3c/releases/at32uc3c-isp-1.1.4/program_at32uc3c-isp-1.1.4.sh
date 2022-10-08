#!/bin/sh
#
# Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
#
# This shell script programs the ISP (flash array), the ISP configuration words
# (User page) and the general-purpose fuse bits.

# Copyright (c) 2009-2018 Microchip Technology Inc. and its subsidiaries.
#
# Subject to your compliance with these terms, you may use Microchip
# software and any derivatives exclusively with Microchip products. 
# It is your responsibility to comply with third party license terms applicable 
# to your use of third party software (including open source software) that 
# may accompany Microchip software.
#
# THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
# WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
# INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
# AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE 
# LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL 
# LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
# SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
# POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
# ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
# RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
# THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.


echo
echo Performing a JTAG Chip Erase command.
avr32program chiperase

echo
echo Programming MCU memory from \`at32uc3c-isp-H.hex\'.
avr32-objcopy -I ihex -O binary at32uc3c-isp-1.1.4.hex at32uc3c-isp-1.1.4.bin
avr32program program -finternal@0x80000000,512Kb -cint -e -v -O0x80000000 -Fbin at32uc3c-isp-1.1.4.bin
rm -f at32uc3c-isp-1.1.4.bin

echo
echo Programming ISP configuration words \(default for EVK1100 i.e. Word2 == 0x929E1424 and Word1 == 0xE11EFFD7\)
avr32program program -finternal@0x80000000 -cint -e -v -O0x808001F8 -Fbin at32uc3c-isp_cfg-1.1.4.dat

echo
echo Programming general-purpose fuse bits.
avr32program writefuses -finternal@0x80000000 gp=0xF877FFFF

echo
echo Resetting MCU.
avr32program reset

pause
