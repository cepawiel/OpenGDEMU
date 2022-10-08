@ECHO Off
setlocal  EnableDelayedExpansion

echo usage:
echo   This program will program a WINC3400 Xplained card plugged into a SamE70XplainedPro card, so long as only one is present
echo   It fills in defaults and calls more specific script files below.
echo.

echo Checking for Python support.
where /q python.exe
if ERRORLEVEL 1 GOTO NOPYTHON
goto :HASPYTHON
:HASPYTHON
python -V > tmpFile 
set /p myvar= < tmpFile 
del tmpFile 
ECHO %myvar%
echo.%myvar%|findstr /C:"2." >nul 2>&1
if not ERRORLEVEL 0 GOTO NOPYTHON
:NOPYTHON
echo Require Python v2.x
exit /b 2

Echo Checking Atmel Studio Installation
python firmware\handler_search.py atsln atmelstudio > atmelstudiopath.txt
if ERRORLEVEL 1 GOTO NOAS
goto :HASAS
:NOAS
echo Require Atmel Studio v7.0
exit /b 2
:HASAS
set /p ASFULLPATH=<atmelstudiopath.txt
REM Typically "C:\Program Files x86\Atmel\Studio\7.0\atmelstudio.exe"
For %%A in (%ASFULLPATH%) do (
    Set ASPATH=%%~dpA
    Set ASEXE=%%~nxA
)
echo Found: %ASPATH%%ASEXE%
echo OK
set ATPROGRAM=%ASPATH%atbackend\atprogram.exe


Echo running: "%ATPROGRAM%" list
set /A edbgCnt=0
set SN=0
for /f "tokens=1-2" %%i in ('"%ATPROGRAM%" list') do (
	if "%%i" == "edbg" (
		set SN=%%j
		set /A edbgCnt+=1
		echo Counting %%i = !edbgCnt!
	) else (
		echo Ignoring %%i
	)
)

if %edbgCnt%==0 (
	echo Cannot find and EDBG boards?
	echo see  '"%ATPROGRAM%" list'
	exit /b 1
)


if %edbgCnt% GTR 1 (
	echo This batch file is unsuitable if more than one EDBG based development board is installed, found %edbgCnt%
	echo Use download_all_sb.bat with options
	echo		edbg
	echo		ATSAME70Q21
	echo		Tools\serial_bridge\same70_xplained_pro_serial_bridge.elf
	echo		3400
	echo		serial number of the dev board attached to the board you wish to program - see '"%ATPROGRAM%" list'
	echo		com port number [0 for auto - when only one edbg board fitted]
	exit /b 1
)

echo Found 1 board

set CHPFAM=3400

echo Calling: download_all_sb.bat
echo With:    edbg ATSAME70Q21 Tools\serial_bridge\same70_xplained_pro_serial_bridge.elf %CHPFAM% %SN% 0
echo.
cd firmware
call download_all_sb.bat edbg ATSAME70Q21 Tools\serial_bridge\same70_xplained_pro_serial_bridge.elf %CHPFAM% %SN% 0



