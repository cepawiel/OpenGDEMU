@echo off

: Copyright (c) 2018-2019 Microchip Technology Inc. and its subsidiaries.
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

rem return 1 if any edbg port is working
set /a PORTCOUNT=0

C:\windows\System32\Wbem\wmic path CIM_LogicalDevice where "Description like 'EDBG Virtual COM Port%'" get /value > tempdbSSS.txt
C:\windows\System32\find.exe /c "DeviceID=COM" < tempdbSSS.txt > tmpportcountSSS.txt
C:\windows\System32\find.exe    "DeviceID=COM" < tempdbSSS.txt > edbgportSSS.txt
set /p PORTCOUNT=<tmpportcountSSS.txt
del tmpportcountSSS.txt
del tempdbSSS.txt
if %PORTCOUNT% GEQ 2 goto errexit
if %PORTCOUNT% EQU 0 goto errexit

:: Must be just one EDBG device
set /p EDBGPORTSSS=<edbgportSSS.txt
set EDBGPORT=%EDBGPORTSSS:~12%
echo Found: COM%EDBGPORT%
del edbgportSSS.txt
if a%1a==aa goto okexit
echo %EDBGPORT% > %1

:okexit
exit /B 0

:errexit
Echo Error: This script will only work with one EDBG device installed.
exit /B 1
