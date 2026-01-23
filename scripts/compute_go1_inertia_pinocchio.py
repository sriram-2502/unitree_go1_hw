#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

try:
    import pinocchio as pin
except Exception as e:
    print("ERROR: pinocchio not available:", e)
    sys.exit(1)


def generate_urdf(xacro_path: Path, out_path: Path) -> None:
    cmd = ["xacro", str(xacro_path)]
    urdf = subprocess.check_output(cmd, text=True)
    out_path.write_text(urdf)


def main() -> None:
    ws_root = Path.home() / "unitree_ws"
    xacro_path = ws_root / "src" / "unitree_ros2" / "go1_description" / "xacro" / "robot.xacro"
    if not xacro_path.exists():
        print("ERROR: xacro not found:", xacro_path)
        sys.exit(2)

    urdf_path = Path("/tmp/go1.urdf")
    generate_urdf(xacro_path, urdf_path)

    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()
    q = pin.neutral(model)
    pin.forwardKinematics(model, data, q)

    total_inertia = pin.Inertia.Zero()
    for i in range(1, model.njoints):
        placement = data.oMi[i] if i < len(data.oMi) else pin.SE3.Identity()
        total_inertia = total_inertia + placement.act(model.inertias[i])

    print("TOTAL_MASS", float(total_inertia.mass))
    print("COM", total_inertia.lever)
    print("INERTIA_WORLD")
    print(total_inertia.inertia)


if __name__ == "__main__":
    main()
