#!/usr/bin/env python3
"""
点云处理工具 (Point Cloud Processing Tool)

功能:
1. 点云变换: 支持平移 (x, y, z) 和旋转 (roll, pitch, yaw)
2. 格式转换: ASCII / Binary / Binary Compressed
3. 信息查看: 显示点云基本信息

依赖: numpy；open3d （强烈推荐，二进制 PCD 必需）
"""

import argparse
import os
import sys

import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def load_matrix4(path):
    """Load a 4x4 row-major matrix. Lines starting with # are ignored."""
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(token) for token in line.split()])
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError(f"{path}: expected 4x4, got {len(rows)} rows")
    return np.asarray(rows, dtype=float)


def euler_to_rotation_matrix(roll, pitch, yaw):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    Rx = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )
    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )
    Rz = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    return Rz @ Ry @ Rx


class PointCloudProcessor:
    def __init__(self):
        self.points = None
        self.has_open3d = HAS_OPEN3D
        self.pcd_o3d = None

    def load(self, filepath):
        print(f"[INFO] Loading: {filepath}")
        if not os.path.exists(filepath):
            print(f"[ERROR] File not found: {filepath}")
            return False

        if self.has_open3d:
            try:
                self.pcd_o3d = o3d.io.read_point_cloud(filepath)
                if self.pcd_o3d.is_empty():
                    print("[WARNING] Open3D loaded an empty cloud; trying ASCII fallback.")
                    return self._load_ascii_fallback(filepath)
                self.points = np.asarray(self.pcd_o3d.points)
                print(f"[SUCCESS] Loaded {len(self.points)} points using Open3D.")
                return True
            except Exception as e:
                print(f"[ERROR] Open3D failed to load file: {e}")
                return False
        return self._load_ascii_fallback(filepath)

    def _load_ascii_fallback(self, filepath):
        print("[INFO] Open3D not available. Attempting to load as ASCII PCD...")
        try:
            header = {}
            points = []
            with open(filepath, "rb") as f:
                while True:
                    line = f.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        break
                    if line.startswith("DATA"):
                        header["DATA"] = line.split()[1]
                        break
                    parts = line.split(maxsplit=1)
                    if len(parts) == 2:
                        header[parts[0]] = parts[1]
                if header.get("DATA") != "ascii":
                    print(f"[ERROR] Unsupported format '{header.get('DATA')}' without Open3D.")
                    print("Please install Open3D: pip3 install open3d")
                    return False
                for line in f:
                    line = line.decode("utf-8", errors="ignore").strip()
                    if line:
                        vals = list(map(float, line.split()))
                        if len(vals) >= 3:
                            points.append(vals[:3])
            self.points = np.array(points)
            print(f"[SUCCESS] Loaded {len(self.points)} points (ASCII mode).")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to load ASCII file: {e}")
            return False

    def info(self):
        if self.points is None or len(self.points) == 0:
            print("[ERROR] No points loaded.")
            return
        mn = self.points.min(axis=0)
        mx = self.points.max(axis=0)
        print(f"[INFO] Points: {len(self.points)}")
        print(f"  X[{mn[0]:.3f}, {mx[0]:.3f}]")
        print(f"  Y[{mn[1]:.3f}, {mx[1]:.3f}]")
        print(f"  Z[{mn[2]:.3f}, {mx[2]:.3f}]")

    def transform(self, tx, ty, tz, roll, pitch, yaw):
        if self.points is None:
            return
        print("[INFO] Applying Transform:")
        print(f"  Translation: [{tx}, {ty}, {tz}]")
        print(
            f"  Rotation (deg): [{np.rad2deg(roll)}, {np.rad2deg(pitch)}, {np.rad2deg(yaw)}]"
        )
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        self.transform_rt(R, np.array([tx, ty, tz], dtype=float))

    def transform_rt(self, rotation, translation):
        if self.points is None:
            return
        self.points = (rotation @ self.points.T).T + translation
        if self.has_open3d and self.pcd_o3d is not None:
            self.pcd_o3d.points = o3d.utility.Vector3dVector(self.points)

    def transform_matrix(self, matrix_4x4):
        print("[INFO] Applying 4x4 matrix (p' = R p + t):")
        print(matrix_4x4)
        self.transform_rt(matrix_4x4[:3, :3], matrix_4x4[:3, 3])

    def save(self, filepath, ascii_format=False):
        print(f"[INFO] Saving to: {filepath}")
        os.makedirs(os.path.dirname(os.path.abspath(filepath)) or ".", exist_ok=True)
        if self.has_open3d and self.pcd_o3d is not None:
            success = o3d.io.write_point_cloud(filepath, self.pcd_o3d, write_ascii=ascii_format)
            if success:
                fmt = "ASCII" if ascii_format else "Binary/Compressed"
                print(f"[SUCCESS] Saved using Open3D ({fmt}).")
                return True
            print("[ERROR] Open3D failed to save file.")
            return False
        return self._save_ascii_fallback(filepath)

    def _save_ascii_fallback(self, filepath):
        try:
            with open(filepath, "w") as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(self.points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(self.points)}\n")
                f.write("DATA ascii\n")
                for p in self.points:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            print("[SUCCESS] Saved as ASCII PCD (Fallback).")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to save file: {e}")
            return False


def parse_args():
    default_in = "/home/ros2_ws/src/mas2027_nav_bringup/pcd/lab3.pcd"
    parser = argparse.ArgumentParser(description="PCD rigid transform / format conversion")
    parser.add_argument("--in", dest="input_pcd", default=default_in, help="input PCD")
    parser.add_argument(
        "--out",
        dest="output_pcd",
        default="/home/ros2_ws/src/mas2027_nav_bringup/pcd/lab3_trans.pcd",
        help="output PCD",
    )
    parser.add_argument("--tx", type=float, default=0.0)
    parser.add_argument("--ty", type=float, default=0.0)
    parser.add_argument("--tz", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0, help="degrees")
    parser.add_argument("--pitch", type=float, default=0.0, help="degrees")
    parser.add_argument("--yaw", type=float, default=0.0, help="degrees")
    parser.add_argument("--ascii", action="store_true", help="write ASCII PCD")
    parser.add_argument("--info", action="store_true", help="print bounds and exit")
    parser.add_argument(
        "--matrix",
        default="",
        help="4x4 row-major text file (p' = R p + t). Used by save_pcd_and_make_map.sh",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("=" * 60)
    print("PCD 点云处理工具")
    print("=" * 60)
    if not HAS_OPEN3D:
        print("[WARNING] Open3D not found; only ASCII PCD is supported.")

    processor = PointCloudProcessor()
    if not processor.load(args.input_pcd):
        sys.exit(1)
    processor.info()
    if args.info:
        return

    if args.matrix:
        try:
            matrix = load_matrix4(args.matrix)
        except Exception as exc:
            print(f"[ERROR] Failed to read --matrix {args.matrix}: {exc}")
            sys.exit(1)
        processor.transform_matrix(matrix)
    elif any([args.tx, args.ty, args.tz, args.roll, args.pitch, args.yaw]):
        processor.transform(
            args.tx,
            args.ty,
            args.tz,
            np.deg2rad(args.roll),
            np.deg2rad(args.pitch),
            np.deg2rad(args.yaw),
        )
    else:
        print("[INFO] No transform; copying / converting format only.")

    save_ascii = args.ascii or (not HAS_OPEN3D)
    if not processor.save(args.output_pcd, ascii_format=save_ascii):
        sys.exit(1)


if __name__ == "__main__":
    main()
