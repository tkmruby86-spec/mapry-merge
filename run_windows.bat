@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo   mapry LiDAR Point Cloud Merger (Windows)
echo ========================================================
echo.

:: 1. Python実行コマンドの探索 (python または py)
set "PY_CMD="
where python >nul 2>nul
if %errorlevel% equ 0 (
    set "PY_CMD=python"
) else (
    where py >nul 2>nul
    if %errorlevel% equ 0 (
        set "PY_CMD=py -3"
    )
)

if "%PY_CMD%"=="" (
    echo [エラー] Python が見つかりませんでした。
    echo.
    echo 以下の点を確認してください：
    echo 1. Python (3.8以上) がインストールされているか
    echo 2. インストール時に「Add Python to PATH」にチェックを入れたか
    echo.
    goto :END
)

:: 2. 必要なライブラリの自動チェック＆インストール
echo [1/2] 必要なライブラリを確認中...
%PY_CMD% -c "import numpy, pandas, scipy" >nul 2>nul
if %errorlevel% neq 0 (
    echo.
    echo 【初回セットアップ】必要なライブラリ（numpy, pandas, scipy）をインストールしています...
    echo 数十秒ほどお待ちください...
    echo.
    %PY_CMD% -m pip install -r "%~dp0requirements.txt"
    if %errorlevel% neq 0 (
        echo.
        echo [エラー] ライブラリのインストールに失敗しました。
        echo インターネット接続を確認してください。
        goto :END
    )
    echo ライブラリの準備が完了しました！
    echo.
) else (
    echo ライブラリの準備OK
    echo.
)

:: 3. スクリプトの実行
echo [2/2] 点群の結合処理を開始します...
echo --------------------------------------------------------

if "%~1"=="" (
    echo ※ データフォルダが直接ドラッグされていないため、
    echo    フォルダ選択画面を開きます（開くまで数秒お待ちください）...
    echo.
    %PY_CMD% "%~dp0mapry_merge.py"
) else (
    echo 入力対象: "%~1"
    echo.
    %PY_CMD% "%~dp0mapry_merge.py" "%~1"
)

if %errorlevel% neq 0 (
    echo.
    echo --------------------------------------------------------
    echo [警告] 処理が途中で終了しました。
    echo 上記のエラー内容を確認してください。
)

:END
echo.
echo ========================================================
echo 画面を閉じるには、キーボードの何かキーを押してください...
pause > nul
