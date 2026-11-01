import os
import sys
import glob
import re
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d


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


def resolve_directories(target_path: Path, output_dir: Path | None = None):
    """
    指定されたパスから入力フォルダ（lidar/）と出力先を特定する。
    直接 '20260619_123157' フォルダを指定されても、親フォルダを指定されても対応可能。
    """
    target_path = target_path.resolve()
    
    # target_path の直下に lidar があるか確認
    if (target_path / "lidar").is_dir():
        base_dir = target_path
    else:
        # 親フォルダが渡された可能性（配下に lidar を持つサブフォルダがあるか検索）
        sub_dirs = [p for p in target_path.iterdir() if p.is_dir() and (p / "lidar").is_dir()]
        if len(sub_dirs) == 1:
            base_dir = sub_dirs[0]
        elif len(sub_dirs) > 1:
            print(f"複数の一致するデータフォルダが見つかりました: {[d.name for d in sub_dirs]}")
            print("処理対象のフォルダを直接指定してください。")
            sys.exit(1)
        else:
            raise FileNotFoundError(f"'lidar' フォルダが見つかりません: {target_path}")

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


def select_folder_gui() -> Path | None:
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


def merge_mapry_data(target_dir: Path, output_dir: Path | None = None):
    target_id, base_dir, lidar_dir, pcd_dir, path_csv, output_ply = resolve_directories(target_dir, output_dir)

    print("==================================================")
    print(f"  mapry Point Cloud Merger")
    print("==================================================")
    print(f"対象データID  : {target_id}")
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

    print(f"合計 {len(pcd_files)} 個のスキャンデータを結合します...")

    all_x, all_y, all_z, all_intensity = [], [], [], []

    # 3. 各PCDファイルの処理
    for i, pcd_path in enumerate(pcd_files, 1):
        if i % 10 == 0 or i == len(pcd_files):
            print(f"[{i}/{len(pcd_files)}] 変換中: {pcd_path.name}")

        with open(pcd_path, "rb") as f:
            while True:
                line = f.readline()
                if not line or line.startswith(b"DATA binary"):
                    break
            raw_data = np.fromfile(f, dtype=POINT_DTYPE)

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

        all_x.append(transformed_pts[:, 0].astype(np.float32))
        all_y.append(transformed_pts[:, 1].astype(np.float32))
        all_z.append(transformed_pts[:, 2].astype(np.float32))
        all_intensity.append(raw_data['intensity'])

    print("全点群データを統合中...")
    all_x = np.concatenate(all_x)
    all_y = np.concatenate(all_y)
    all_z = np.concatenate(all_z)
    all_intensity = np.concatenate(all_intensity)

    total_points = len(all_x)
    print(f"結合完了: 総点数 {total_points:,} 点")

    # 4. PLY出力
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

    out_data = np.empty(total_points, dtype=OUT_DTYPE)
    out_data['x'] = all_x
    out_data['y'] = all_y
    out_data['z'] = all_z
    out_data['intensity'] = all_intensity

    print(f"PLYファイルを出力中: {output_ply}")
    with open(output_ply, "wb") as f_out:
        f_out.write(ply_header)
        out_data.tofile(f_out)

    print("--------------------------------------------------")
    print(f"【処理成功】出力が完了しました:")
    print(f" -> {output_ply}")
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
        print("データフォルダが指定されていません。フォルダ選択ダイアログを開きます...")
        gui_path = select_folder_gui()
        if gui_path:
            target_dir = gui_path
        else:
            try:
                input_str = input("対象データフォルダのパスを入力してください: ").strip().strip('\'"')
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
        print(f"\nエラーが発生しました: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
