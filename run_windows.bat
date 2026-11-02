@echo off
title mapry Point Cloud Merger
cd /d "%~dp0"

echo ========================================================
echo   mapry LiDAR Point Cloud Merger (Windows)
echo ========================================================
echo.

:: 1. Search for Python
set PY=

python --version >nul 2>&1
if not errorlevel 1 (
    set PY=python
    goto :PYTHON_FOUND
)

py -3 --version >nul 2>&1
if not errorlevel 1 (
    set PY=py -3
    goto :PYTHON_FOUND
)

:: Search standard install paths if PATH was not set
for /d %%D in ("%LocalAppData%\Programs\Python\Python3*") do (
    if exist "%%~fD\python.exe" (
        set "PY=%%~fD\python.exe"
        goto :PYTHON_FOUND
    )
)

for /d %%D in ("C:\Python3*") do (
    if exist "%%~fD\python.exe" (
        set "PY=%%~fD\python.exe"
        goto :PYTHON_FOUND
    )
)

for /d %%D in ("%ProgramFiles%\Python3*") do (
    if exist "%%~fD\python.exe" (
        set "PY=%%~fD\python.exe"
        goto :PYTHON_FOUND
    )
)

:PYTHON_NOT_FOUND
echo [ERROR] Python was not found on this computer.
echo.
echo Please install Python 3.8 or higher from python.org.
echo Important: Check "Add python.exe to PATH" during installation.
echo.
pause
exit /b 1

:PYTHON_FOUND
echo Python found: %PY%
echo.

:: 2. Check and Install Libraries
echo Checking required libraries...
%PY% -c "import numpy, pandas, scipy" >nul 2>&1
if errorlevel 1 (
    echo.
    echo Installing required packages: numpy, pandas, scipy...
    echo Please wait a minute...
    echo.
    %PY% -m pip install -r requirements.txt
    if errorlevel 1 (
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

if errorlevel 1 (
    echo.
    echo --------------------------------------------------------
    echo [ERROR] Process finished with errors.
)

echo.
echo ========================================================
echo Press any key to exit...
pause >nul
