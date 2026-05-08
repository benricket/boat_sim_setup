#!/usr/bin/env python3
import json
import math
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


def euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw to quaternion (x, y, z, w)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def load_config(path: Path) -> dict:
    suffix = path.suffix.lower()

    if suffix == ".json":
        return json.loads(path.read_text())

    if suffix in {".yaml", ".yml"}:
        if yaml is None:
            raise RuntimeError(
                "PyYAML is not installed. Install it with: pip install pyyaml"
            )
        return yaml.safe_load(path.read_text())

    raise RuntimeError(f"Unsupported config format: {path}")


def build_request(name: str, model_ref: str, x: float, y: float, z: float,
                  roll: float, pitch: float, yaw: float) -> str:
    qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)

    # If model_ref ends with .sdf or .urdf, treat it as a filename/path.
    # Otherwise treat it as a model name in GZ_SIM_RESOURCE_PATH.
    if model_ref.endswith(".sdf") or model_ref.endswith(".urdf") or "/" in model_ref:
        model_field = f'sdf_filename: "{model_ref}"'
    else:
        model_field = f'sdf_filename: "{model_ref}"'

    return f"""
name: "{name}"
{model_field}
pose {{
  position {{ x: {x} y: {y} z: {z} }}
  orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }}
}}
""".strip()


def spawn_one(world: str, req: str) -> None:
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        raise RuntimeError(
            f"Spawn failed.\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )

    print(result.stdout.strip() or "Spawn request sent successfully.")


def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} obstacles.yaml", file=sys.stderr)
        return 2

    arg_num = len(sys.argv) - 1 

    for i in range(arg_num):
        config_path = Path(sys.argv[i+1])
        config = load_config(config_path)

        world = config["world"]
        model_ref = config["model"]
        obstacles = config["obstacles"]

        for obs in obstacles:
            req = build_request(
                name=obs["name"],
                model_ref=model_ref,
                x=float(obs["x"]),
                y=float(obs["y"]),
                z=float(obs["z"]),
                roll=float(obs.get("roll", 0.0)),
                pitch=float(obs.get("pitch", 0.0)),
                yaw=float(obs.get("yaw", 0.0)),
            )
            print(f"Spawning {obs['name']}...")
            spawn_one(world, req)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())