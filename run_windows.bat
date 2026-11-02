@echo off
title mapry Point Cloud Merger
cd /d "%~dp0"

echo ========================================================
echo   mapry LiDAR Point Cloud Merger (Windows)
echo ========================================================
echo.

:: 1. Check Python
set PY=python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    py -3 --version >nul 2>&1
    if %errorlevel% neq 0 (
        echo [ERROR] Python is not installed or not in PATH.
        echo.
        echo Please install Python (3.8+) and make sure to check:
        echo   [x] Add python.exe to PATH
        echo.
        pause
        exit /b 1
    )
    set PY=py -3
)

echo Python found.
echo.

:: 2. Check and Install Libraries
echo Checking required libraries...
%PY% -c "import numpy, pandas, scipy" >nul 2>&1
if %errorlevel% neq 0 (
    echo.
    echo Installing required packages (numpy, pandas, scipy)...
    echo Please wait a minute...
    echo.
    %PY% -m pip install -r requirements.txt
    if %errorlevel% neq 0 (
        echo.
        echo [ERROR] Failed to install libraries.
        echo Please check your internet connection.
        echo.
        pause
        exit /b 1
    )
    echo Libraries installed successfully.
    echo.
)

:: 3. Run mapry_merge.py
echo Starting point cloud merger...
echo --------------------------------------------------------

if "%~1"=="" (
    %PY% mapry_merge.py
) else (
    %PY% mapry_merge.py "%~1"
)

if %errorlevel% neq 0 (
    echo.
    echo --------------------------------------------------------
    echo [ERROR] Process finished with errors.
)

echo.
echo ========================================================
echo Press any key to exit...
pause >nul
