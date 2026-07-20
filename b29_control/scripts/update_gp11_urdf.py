#!/usr/bin/env python3
"""
update_gp11_urdf.py

从 b29_locomotion 重新生成并更新 gp11_urdf 目录中的训练侧换根 URDF。

用法：
  python3 update_gp11_urdf.py
  python3 update_gp11_urdf.py --b29_locomotion /path/to/b29_locomotion
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_DEST_DIR  = _THIS_DIR.parent / "models" / "gp11_urdf"

_GENERATED_FILES = {
    "left":  "gp11_scene_left_gripper_root_coacd.urdf",
    "right": "gp11_scene_right_gripper_root_coacd.urdf",
}


def find_b29_locomotion(hint: str = "") -> Path:
    candidates = [
        Path(hint) if hint else None,
        Path.home() / "usetest" / "RL" / "b29_locomotion",
    ]
    for c in candidates:
        if c and c.exists() and (c / "legged_gym").exists():
            return c
    raise FileNotFoundError(
        "找不到 b29_locomotion，请通过 --b29_locomotion 指定路径"
    )


def main():
    parser = argparse.ArgumentParser(description="更新训练侧换根 URDF")
    parser.add_argument("--b29_locomotion", default="", help="b29_locomotion 仓库路径")
    parser.add_argument("--dry_run", action="store_true", help="只显示操作，不实际执行")
    args = parser.parse_args()

    b29_loco = find_b29_locomotion(args.b29_locomotion)
    generate_script = b29_loco / "legged_gym/resources/robots/gp11/urdf/generate_gp11_gripper_root_urdf.py"
    source_dir = b29_loco / "legged_gym/resources/robots/gp11/urdf"

    if not generate_script.exists():
        print(f"ERROR: 生成脚本不存在: {generate_script}")
        sys.exit(1)

    print(f"b29_locomotion: {b29_loco}")
    print(f"生成脚本:        {generate_script}")
    print(f"目标目录:        {_DEST_DIR}")
    print()

    if not args.dry_run:
        print("运行生成脚本...")
        result = subprocess.run(
            [sys.executable, str(generate_script)],
            cwd=str(source_dir),
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"ERROR: 生成脚本失败:\n{result.stderr}")
            sys.exit(1)
        print("生成完成")
        print()

    _DEST_DIR.mkdir(parents=True, exist_ok=True)
    for side, filename in _GENERATED_FILES.items():
        src = source_dir / filename
        dst = _DEST_DIR / filename
        if not src.exists():
            print(f"WARNING: 源文件不存在，跳过: {src}")
            continue
        if args.dry_run:
            print(f"[dry_run] 将复制: {src.name} → {dst}")
        else:
            shutil.copy2(src, dst)
            print(f"已更新: {dst.name}")

    if not args.dry_run:
        print()
        print("URDF 更新完成。请重启推理节点和键盘节点使其生效。")


if __name__ == "__main__":
    main()
