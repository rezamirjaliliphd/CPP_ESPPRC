@echo off
setlocal

REM Try to detect g++
where g++ >nul 2>nul
if %errorlevel%==0 (
    echo [INFO] MinGW detected, using g++ to build.

    REM Clean previous build
    if exist build (
        rmdir /s /q build
    )
    mkdir build
    cd build

    REM Compile all cpp files using g++
    g++ ../*.cpp -std=c++17 -O2 -o CPP_ESPPRC.exe
    if %errorlevel% neq 0 (
        echo [ERROR] Compilation failed with g++
        exit /b %errorlevel%
    )

    echo [INFO] Running program...
    CPP_ESPPRC.exe
) else (
    echo [INFO] MinGW not found, using CMake with Visual Studio.

    REM Clean and rebuild with CMake
    if exist build (
        rmdir /s /q build
    )
    mkdir build
    cd build

    cmake .. -G "Visual Studio 17 2022" -A x64
    if %errorlevel% neq 0 (
        echo [ERROR] CMake configuration failed
        exit /b %errorlevel%
    )

    cmake --build . --config Release
    if %errorlevel% neq 0 (
        echo [ERROR] Build failed
        exit /b %errorlevel%
    )

    echo [INFO] Running program...
    .\Release\CPP_ESPPRC.exe
)

endlocal
