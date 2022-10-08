@ECHO off
setlocal EnableDelayedExpansion

:: %1 type of bus
:: %2 (3400)
:: %3 (aardvark_serial_number or 0)
:: %4 com port for edbg usb device or 0

set TOOL=%1
set TGTIN=%2
set AADVSNIN=%3
set PRTIN=%4

echo Mode %TOOL%
if /I "%TOOL%" == "UART"  Goto contine_UART
if /I "%TOOL%" == "I2C"   Goto contine_I2C

goto usage

:contine_I2C
Set  FMode=debug_i2c
set Mode=I2C
set ptool=winc_programmer_i2c.exe
goto START

:contine_UART
Set  FMode=debug_uart
set Mode=UART
set ptool=winc_programmer_uart.exe  -p \\.\COM%PRTIN%
goto START

:START

if /I "a%AADVSNIN%a" == "aa" (
	set AARDVARK=0
) else (
	set AARDVARK=%AADVSNIN%
)

echo Chip %TGTIN%
if /I "%TGTIN%" == "3400"  Goto chip3400

:USAGE
echo Usage %0 (I2C-UART) (3400) (aardvark_serial_number or 0) (comport or 0 for I2C)
@ECHO %CMDCMDLINE% | FIND /I /C "/C" > NUL && PAUSE
exit /b 2

:chip3400
set VARIANT=3400
set CHPFAM=3400
set PFW=programmer_firmware.bin
goto parmsok

:parmsok

pushd Tools\image_builder_downloader
echo Downloading Image...
set TGT_CHIP=3400

echo Building flash images (prog format)
echo image_tool.exe -c ..\..\flash_image.config -o m2m_image_%VARIANT%.bin -of prog
image_tool.exe -c ..\..\flash_image.config -o m2m_image_%VARIANT%.bin -of prog
echo Call %ptool% -d winc%CHPFAM% -i m2m_image_%VARIANT%.bin -if prog -w -r
echo      -pfw ..\..\programmer_firmware\release%VARIANT%\%PFW%
%ptool% -d winc%CHPFAM% -i m2m_image_%VARIANT%.bin -if prog -w -r -pfw ..\..\programmer_firmware\release%VARIANT%\%PFW%
popd
if %ERRORLEVEL% NEQ 0 GOTO FAILED
goto SUCCESS

:FAILED
echo             *************** Failed to download *****************
echo     #######################################################################
echo     ##                                                                   ##
echo     ##                    ########    ###     ####  ##                   ##
echo     ##                    ##         ## ##     ##   ##                   ##
echo     ##                    ##        ##   ##    ##   ##                   ##
echo     ##                    ######   ##     ##   ##   ##                   ##
echo     ##                    ##       #########   ##   ##                   ##
echo     ##                    ##       ##     ##   ##   ##                   ##
echo     ##                    ##       ##     ##  ####  ########             ##
echo     ##                                                                   ##
echo     #######################################################################
pause
exit /b 1

:SUCCESS
echo              *************** Download worked *****************
echo     #######################################################################
echo     ##                                                                   ##
echo     ##                 ########     ###     ######   ######              ##
echo     ##                 ##     ##   ## ##   ##    ## ##    ##             ##
echo     ##                 ##     ##  ##   ##  ##       ##                   ##
echo     ##                 ########  ##     ##  ######   ######              ##
echo     ##                 ##        #########       ##       ##             ##
echo     ##                 ##        ##     ## ##    ## ##    ##             ##
echo     ##                 ##        ##     ##  ######   ######              ##
echo     ##                                                                   ##
echo     #######################################################################
pause
exit /b 0
