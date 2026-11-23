@echo off
REM One-click PC test of subsystem 3 terrain maths (Zi Siang) (no Pico needed).
REM Double-click this file. It compiles imu.c with gcc and runs 35 checks.

setlocal
cd /d "%~dp0"

set GCC=gcc
where gcc >nul 2>nul
if errorlevel 1 (
    if exist "C:\MinGW\mingw64\bin\gcc.exe" (
        set GCC=C:\MinGW\mingw64\bin\gcc.exe
    ) else if exist "C:\msys64\mingw64\bin\gcc.exe" (
        set GCC=C:\msys64\mingw64\bin\gcc.exe
    ) else (
        echo.
        echo gcc was not found. Install it with:  winget install BrechtSanders.WinLibs.POSIX.UCRT
        echo or just run the same test inside WSL Ubuntu:
        echo    cd testing/subsystem3/buddy4_host ^&^& make run
        echo.
        pause
        exit /b 1
    )
)

echo Building the test with %GCC% ...
%GCC% -std=c99 -Wall -Wextra -DIMU_TEST_HOOKS -I..\..\..\app_program\subsystem3 ..\..\..\app_program\subsystem3\imu.c imu_host_test.c -o imu_host_test.exe
if errorlevel 1 (
    echo.
    echo BUILD FAILED - send the text above.
    pause
    exit /b 1
)

echo.
imu_host_test.exe
echo.
echo Copy this window (right-click - Select All - Enter) into
echo testing/subsystem3/logs/ if you want it saved.
pause
