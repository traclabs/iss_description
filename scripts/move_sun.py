#!/usr/bin/env python3
import subprocess
import math
import argparse

TARGET_NAME = "sun_model"
WORLD_NAME = "default"
ORBIT_RADIUS = 10000.0

def get_x_axis_rotation_quaternion(y, z):
    """
    Calculates the 'Roll' (Rotation around X-axis) needed
    for a light at (0, y, z) to point its -Z axis at (0,0,0).
    """
    # 1. Calculate the angle from the vertical (Zenith)
    # math.atan2(y, z) gives the angle of the vector relative to vertical.
    # Since the light points Down (-Z), we negate the angle to rotate it correctly.
    roll = -math.atan2(y, z)

    # 2. Convert Euler Roll to Quaternion
    # Rotation around X-axis: q = [sin(theta/2), 0, 0, cos(theta/2)]
    qx = math.sin(roll / 2.0)
    qw = math.cos(roll / 2.0)

    # Y and Z components of the quaternion are 0 for pure X-axis rotation
    return (qx, 0.0, 0.0, qw)

def set_sun_pose(angle_degrees):
    rad = math.radians(angle_degrees)

    # --- CHANGED: ORBIT IN Y-Z PLANE (Rotate around X) ---
    x = 0
    y = ORBIT_RADIUS * math.sin(rad)
    z = ORBIT_RADIUS * math.cos(rad)

    # Safety: don't go underground
    if z < 100: z = 100

    # Calculate Orientation (Roll instead of Pitch)
    qx, qy, qz, qw = get_x_axis_rotation_quaternion(y, z)

    print(f"Moving {TARGET_NAME} (X-Axis Rot) -> Angle {angle_degrees}° | Pos [0, {y:.0f}, {z:.0f}]")

    req_str = (
        f'name: "{TARGET_NAME}", '
        f'position: {{x: {x:.1f}, y: {y:.1f}, z: {z:.1f}}}, '
        f'orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}'
    )

    cmd = [
        "gz", "service",
        "-s", f"/world/{WORLD_NAME}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--req", req_str
    ]
    subprocess.run(cmd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, default=0.0)
    args = parser.parse_args()
    set_sun_pose(args.angle)