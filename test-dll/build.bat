@echo off
REM build.bat - Builds FATestDll.dll and Injector.exe using MSYS2 MinGW32
REM Run this from the test-dll\ directory

setlocal

REM -----------------------------------------------------------------------
REM Find MinGW32 compiler (tries common MSYS2 install paths)
REM -----------------------------------------------------------------------
set GCC=
if exist "C:\msys64\mingw32\bin\i686-w64-mingw32-g++.exe" (
    set GCC=C:\msys64\mingw32\bin\i686-w64-mingw32-g++.exe
    set "PATH=C:\msys64\mingw32\bin;C:\msys64\usr\bin;%PATH%"
)
if exist "C:\msys2\mingw32\bin\i686-w64-mingw32-g++.exe" (
    set GCC=C:\msys2\mingw32\bin\i686-w64-mingw32-g++.exe
    set "PATH=C:\msys2\mingw32\bin;C:\msys2\usr\bin;%PATH%"
)

if "%GCC%"=="" (
    echo [ERROR] MinGW32 not found!
    echo.
    echo Install MSYS2 first:
    echo   winget install MSYS2.MSYS2
    echo.
    echo Then open MSYS2 MinGW32 shell and run:
    echo   pacman -S mingw-w64-i686-gcc
    echo.
    pause
    exit /b 1
)

echo [*] Using compiler: %GCC%
echo [*] Building FATestDll.dll...

"%GCC%" -m32 -O0 -g -std=c++17 -Wall ^
    -static -static-libgcc -static-libstdc++ ^
    -I../include -I. ^
    -DBUILDING_FA_TEST_DLL ^
    -shared ^
    -o FATestDll.dll ^
    dllmain.cpp collision.cpp ^
    -lkernel32 -luser32 -lpsapi

if errorlevel 1 (
    echo [ERROR] FATestDll.dll build failed!
    pause
    exit /b 1
)
echo [OK] FATestDll.dll built successfully

echo [*] Building Injector.exe...

"%GCC%" -m32 -O2 -std=c++17 ^
    -static -static-libgcc -static-libstdc++ ^
    -o Injector.exe ^
    injector.cpp ^
    -lkernel32

if errorlevel 1 (
    echo [ERROR] Injector.exe build failed!
    pause
    exit /b 1
)
echo [OK] Injector.exe built successfully

echo.
echo =====================================================
echo  BUILD SUCCESSFUL
echo =====================================================
echo  FATestDll.dll  ^<-- inject this into ForgedAlliance.exe
echo  Injector.exe   ^<-- run this to inject automatically
echo.
echo  How to test:
echo  1. Start ForgedAlliance.exe (via FAF client)
echo  2. Double-click Injector.exe (run as Admin if needed)
echo  3. Check FATest.log in the game directory
echo =====================================================
echo.
pause
