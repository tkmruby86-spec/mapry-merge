@echo off
chcp 65001 > nul
setlocal

echo ========================================================
echo   mapry LiDAR Point Cloud Merger (Windows)
echo ========================================================

:: Pythonの確認
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [エラー] Pythonがインストールされていないか、PATHが通っていません。
    echo Python 3.8 以上をインストールしてください。
    pause
    exit /b 1
)

:: 引数（ドラッグ＆ドロップされたフォルダ）があるか確認
if "%~1"=="" (
    echo データフォルダをこの .bat ファイルにドラッグ＆ドロップするか、
    echo 起動後に表示されるダイアログでフォルダを選択してください。
    echo.
    python "%~dp0mapry_merge.py"
) else (
    echo 対象フォルダ: %~1
    python "%~dp0mapry_merge.py" "%~1"
)

echo.
pause
