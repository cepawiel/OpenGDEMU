@echo off

: Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
:
: \asf_license_start
:
: \page License
:
: Subject to your compliance with these terms, you may use Microchip
: software and any derivatives exclusively with Microchip products.
: It is your responsibility to comply with third party license terms applicable
: to your use of third party software (including open source software) that
: may accompany Microchip software.
:
: THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
: WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
: INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
: AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
: LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
: LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
: SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
: POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
: ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
: RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
: THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
:
: \asf_license_stop

REM echo Call with taget chip 1500 or 3400 to build a compound full and OTA image.
set TGT_CHIP=3400

echo Building flash images (prog format)
echo firmware\image_tool.exe -c flash_image.config -o firmware\m2m_image_%TGT_CHIP%.bin -of prog
firmware\image_tool.exe -c flash_image.config -o firmware\m2m_image_%TGT_CHIP%.bin -of prog
REM echo firmware\image_tool.exe -c firmware\flash_image.config -o ota_firmware\m2m_ota_%TGT_CHIP%.bin -of winc_ota -s ota
REM firmware\image_tool.exe -c firmware\flash_image.config -o ota_firmware\m2m_ota_%TGT_CHIP%.bin -of winc_ota -s ota
