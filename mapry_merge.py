from __future__ import annotations
import os
import sys
import glob
import re
import argparse
from pathlib import Path
from typing import Optional, Tuple
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d

# Windows環境でのコンソール文字化け・Unicodeエラー防止
if sys.platform == "win32":
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(errors="replace")
    except Exception:
        pass


# PCDヘッダー定義（24バイト構造）
POINT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4'),
    ('timestamp_sec', '<u4'),
    ('timestamp_nsec', '<u4')
])

OUT_DTYPE = np.dtype([
    ('x', '<f4'),
    ('y', '<f4'),
    ('z', '<f4'),
    ('intensity', '<f4')
])


def extract_number(path: Path) -> int:
    """ファイル名から末尾の数値を抽出してソートキーにする"""
    nums = re.findall(r'\d+', path.name)
    return int(nums[-1]) if nums else 999999


def find_lidar_base(target_path: Path) -> Optional[Path]:
    """
    指定パス、またはその親・子階層から 'lidar' フォルダを持つ基底フォルダを探索する。
    例:
      - 2026xxxx_xxxx/
      - 2026xxxx_xxxx/lidar/
      - 2026xxxx_xxxx/lidar/pointcloud/
      - 2026xxxx_xxxx/lidar/pointcloud/xxx.pcd
    のいずれが指定されても 2026xxxx_xxxx を自動特定する。
    """
    cur = target_path.resolve()
    if cur.is_file():
        cur = cur.parent

    # 1. 自身または親階層を遡って探す
    check_dir = cur
    for _ in range(5):
        if (check_dir / "lidar").is_dir():
            return check_dir
        if check_dir.name == "lidar":
            return check_dir.parent
        if check_dir.parent == check_dir:
            break
        check_dir = check_dir.parent

    # 2. 直下のサブディレクトリから探す
    if cur.is_dir():
        sub_dirs = [p for p in cur.iterdir() if p.is_dir() and (p / "lidar").is_dir()]
        if len(sub_dirs) == 1:
            return sub_dirs[0]
        elif len(sub_dirs) > 1:
            print(f"複数の一致するデータフォルダが見つかりました: {[d.name for d in sub_dirs]}")
            print("処理したい特定のフォルダを指定してください。")
            sys.exit(1)

    return None


def resolve_directories(target_path: Path, output_dir: Optional[Path] = None):
    """
    指定されたパスから入力フォルダ（lidar/）と出力先を特定する。
    """
    base_dir = find_lidar_base(target_path)
    if not base_dir:
        raise FileNotFoundError(
            f"指定された場所から 'lidar' フォルダが見つかりませんでした。\n"
            f"指定パス: {target_path}\n"
            f"フォルダ内に 'lidar' (中に local_path.csv と pointcloud フォルダ) があるか確認してください。"
        )

    lidar_dir = base_dir / "lidar"
    pcd_dir = lidar_dir / "pointcloud"
    path_csv = lidar_dir / "local_path.csv"

    if not path_csv.is_file():
        raise FileNotFoundError(f"軌跡ファイルが見つかりません: {path_csv}")
    if not pcd_dir.is_dir():
        raise FileNotFoundError(f"点群フォルダが見つかりません: {pcd_dir}")

    target_id = base_dir.name
    out_dir = (output_dir.resolve() if output_dir else base_dir.parent)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_ply = out_dir / f"forest_combined_{target_id}.ply"

    return target_id, base_dir, lidar_dir, pcd_dir, path_csv, output_ply


def select_folder_gui() -> Optional[Path]:
    """GUIファイル選択ダイアログ（Windows等で引数なし起動された場合のフォールバック）"""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        selected = filedialog.askdirectory(title="mapryのデータフォルダ（2026xxxx_xxxx）を選択してください")
        root.destroy()
        if selected:
            return Path(selected)
    except Exception:
        pass
    return None


def get_pcd_point_count(pcd_path: Path) -> int:
    """PCDヘッダーから点数を高速に取得する"""
    with open(pcd_path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith(b"POINTS"):
                parts = line.split()
                if len(parts) >= 2:
                    return int(parts[1])
            if line.startswith(b"DATA"):
                break
    return 0


def merge_mapry_data(target_dir: Path, output_dir: Optional[Path] = None):
    target_id, base_dir, lidar_dir, pcd_dir, path_csv, output_ply = resolve_directories(target_dir, output_dir)

    print("==================================================")
    print(f"  mapry Point Cloud Merger")
    print("==================================================")
    print(f"対象データID    : {target_id}")
    print(f"入力ディレクトリ: {base_dir}")
    print(f"出力ファイル    : {output_ply}")
    print("--------------------------------------------------")

    # 1. 軌跡データの読み込みと補間器の生成
    print("軌跡データ (local_path.csv) を読み込み中...")
    df_path = pd.read_csv(path_csv)
    
    required_cols = {'timestamp', 'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w'}
    missing = required_cols - set(df_path.columns)
    if missing:
        raise ValueError(f"local_path.csv に必要なカラムが不足しています: {missing}")

    path_times = df_path['timestamp'].values
    positions = df_path[['pos_x', 'pos_y', 'pos_z']].values
    quats = df_path[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values

    print(f"軌跡記録数: {len(path_times):,} 点 (時間範囲: {path_times[0]:.2f} - {path_times[-1]:.2f})")

    # 姿勢（Slerp）と位置（1D補間）の関数生成
    rotations = R.from_quat(quats)
    slerp = Slerp(path_times, rotations)
    interp_pos = interp1d(path_times, positions, axis=0, bounds_error=False, fill_value="extrapolate")

    # 2. PCDファイル検索・ソート
    pcd_files = sorted(pcd_dir.glob("*.pcd"), key=extract_number)
    if not pcd_files:
        raise FileNotFoundError(f"PCDファイルが見つかりません: {pcd_dir}")

    # 事前に総点数をヘッダーから計算（超高速）
    print("点群メタデータを解析中...")
    total_points = sum(get_pcd_point_count(p) for p in pcd_files)
    print(f"処理対象: {len(pcd_files)} 個のスキャンファイル (総点数: {total_points:,} 点)")

    # 3. PLYヘッダーを先行書き込み
    ply_header = (
        f"ply\n"
        f"format binary_little_endian 1.0\n"
        f"element vertex {total_points}\n"
        f"property float x\n"
        f"property float y\n"
        f"property float z\n"
        f"property float intensity\n"
        f"end_header\n"
    ).encode('ascii')

    # 4. ストリーミング書き出し（メモリ消費を抑え、巨大点群でもクラッシュしない方式）
    print(f"PLYファイルへストリーミング変換中: {output_ply.name}")
    processed_points = 0

    with open(output_ply, "wb") as f_out:
        f_out.write(ply_header)

        for i, pcd_path in enumerate(pcd_files, 1):
            with open(pcd_path, "rb") as f_in:
                while True:
                    line = f_in.readline()
                    if not line or line.startswith(b"DATA binary"):
                        break
                raw_data = np.fromfile(f_in, dtype=POINT_DTYPE)

            if len(raw_data) == 0:
                continue

            # タイムスタンプ復元 (秒 + ナノ秒)
            point_times = raw_data['timestamp_sec'].astype(np.float64) + raw_data['timestamp_nsec'].astype(np.float64) * 1e-9

            # 軌跡の有効範囲内にクランプ
            valid_times = np.clip(point_times, path_times[0], path_times[-1])

            # タイムスタンプに応じた位置・回転の取得
            interp_rot = slerp(valid_times)
            pos = interp_pos(valid_times)

            pts = np.stack([raw_data['x'], raw_data['y'], raw_data['z']], axis=1)

            # 局所座標系からグローバル座標系への回転・平行移動
            transformed_pts = interp_rot.apply(pts) + pos

            # 出力用バッファに詰めてファイルへ直接追記書き込み（メモリを即時解放）
            batch_data = np.empty(len(raw_data), dtype=OUT_DTYPE)
            batch_data['x'] = transformed_pts[:, 0].astype(np.float32)
            batch_data['y'] = transformed_pts[:, 1].astype(np.float32)
            batch_data['z'] = transformed_pts[:, 2].astype(np.float32)
            batch_data['intensity'] = raw_data['intensity']

            batch_data.tofile(f_out)

            processed_points += len(raw_data)
            progress = (processed_points / total_points * 100) if total_points > 0 else 0
            print(f"[{i}/{len(pcd_files)}] {pcd_path.name} 完了 ({len(raw_data):,} 点 / 累計進捗: {progress:.1f}%)")

            # メモリ解放
            del raw_data, transformed_pts, batch_data

    print("--------------------------------------------------")
    print(f"【処理成功】結合が正常に完了しました！")
    print(f"出力ファイル: {output_ply} (総点数: {processed_points:,} 点)")
    print("==================================================")


def main():
    parser = argparse.ArgumentParser(
        description="mapry LiDARで取得した分割PCDデータと軌跡CSVを統合し、単一のPLY点群ファイルを作成します。"
    )
    parser.add_argument(
        "input_dir",
        nargs="?",
        type=Path,
        default=None,
        help="対象のデータフォルダパス（例: D:\\mapry_la03_1\\20260619_123157）"
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=Path,
        default=None,
        help="PLYファイルの保存先ディレクトリ（省略時は入力フォルダの親ディレクトリ）"
    )

    args = parser.parse_args()

    target_dir = args.input_dir
    if target_dir is None:
        # 引数が与えられていない場合はGUIダイアログまたはコンソール入力を試みる
        print("データフォルダが直接指定されていません。フォルダ選択ダイアログを開きます...")
        gui_path = select_folder_gui()
        if gui_path:
            target_dir = gui_path
        else:
            try:
                input_str = input("対象データフォルダのパスを入力してください（ドラッグ＆ドロップでも可）: ").strip().strip('\'"')
                if input_str:
                    target_dir = Path(input_str)
            except (KeyboardInterrupt, EOFError):
                sys.exit(0)

    if not target_dir or not target_dir.exists():
        print("エラー: 有効なデータフォルダが指定されませんでした。")
        print("使い方: python mapry_merge.py <データフォルダのパス>")
        sys.exit(1)

    try:
        merge_mapry_data(target_dir, args.output_dir)
    except Exception as e:
        print(f"\n【エラーが発生しました】: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
