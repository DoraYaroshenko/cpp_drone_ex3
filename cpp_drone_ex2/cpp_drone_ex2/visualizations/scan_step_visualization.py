import json
from pathlib import Path
import argparse
import matplotlib.pyplot as plt
import numpy as np


# ---------- helpers ----------

def orientation_to_direction(h_deg, v_deg):

    h = np.deg2rad(h_deg)
    v = np.deg2rad(v_deg)

    dx = np.cos(v) * np.cos(h)
    dy = np.cos(v) * np.sin(h)
    dz = np.sin(v)

    return np.array([dx, dy, dz])


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize scan step.")
    parser.add_argument("--log", required=True, type=Path, help="Path to drone logs json/jsonl")
    parser.add_argument("--output_map", required=True, type=Path, help="Path to output map npy")
    parser.add_argument("--capabilities", required=True, type=Path, help="Path to drone capabilities json")
    parser.add_argument("--step", required=True, type=int, help="Scan step ID")
    parser.add_argument("--resolution", type=float, default=10.0, help="Voxel resolution in cm")

    args = parser.parse_args()

    log_path = args.log
    map_path = args.output_map
    capabilities_path = args.capabilities
    SCAN_STEP = args.step
    resolution = args.resolution

    # ---------- load ----------

    data = {"movements": [], "scans": [], "voxels": []}
    with open(log_path) as f:
        for line in f:
            obj = json.loads(line)
            data[obj["type"] + "s"].append(obj)
    logs = data

    with open(capabilities_path) as f:
        capabilities = json.load(f)

    voxel_map_zyx = np.load(map_path)

    # z,y,x -> x,y,z
    voxel_map = np.transpose(voxel_map_zyx, (2, 1, 0))

    lidar_max_range = capabilities["lidar"]["beam_length_max"]

    movement_by_step = {
        m["step_id"]: m
        for m in logs["movements"]
    }

    scan_by_step = {
        s["step_id"]: s
        for s in logs["scans"]
    }

    # ---------- choose scan step ----------

    scan = scan_by_step[SCAN_STEP]
    movement = movement_by_step[SCAN_STEP-1]

    drone_position = np.array([
        movement["x_cm"],
        movement["y_cm"],
        movement["z_cm"]
    ])

    absolute_h = (
        movement["h_angle_deg"] +
        scan["relative_h_angle_deg"]
    )

    absolute_v = (
        movement["v_angle_deg"] +
        scan["relative_v_angle_deg"]
    )

    direction = orientation_to_direction(
        absolute_h,
        absolute_v
    )

    ray_end = (
        drone_position +
        direction * lidar_max_range
    )

    # ---------- only voxels changed by this scan ----------

    scan_voxels = [
        v for v in logs["voxels"]
        if v["step_id"] == SCAN_STEP
    ]

    filled = np.zeros(voxel_map.shape, dtype=bool)

    colors = np.empty(voxel_map.shape, dtype=object)

    for v in scan_voxels:

        x = v["x"]
        y = v["y"]
        z = v["z"]

        filled[x, y, z] = True

        if v["value"] == 1:
            colors[x, y, z] = "red"

        elif v["value"] == 0:
            colors[x, y, z] = "green"

        elif v["value"] == -1:
            colors[x, y, z] = "blue"

        elif v["value"] == -2:
            colors[x, y, z] = "gray"

        else:
            colors[x, y, z] = "purple"

    # ---------- plot ----------

    fig = plt.figure(figsize=(12, 10))

    ax = fig.add_subplot(111, projection="3d")

    # grid coordinates
    x_grid, y_grid, z_grid = np.indices((voxel_map.shape[0]+1, voxel_map.shape[1]+1, voxel_map.shape[2]+1)) * resolution

    # full cubes
    # ---------- faint background grid ----------

    grid = np.ones(voxel_map.shape, dtype=bool)

    ax.voxels(
        x_grid, y_grid, z_grid,
        grid,
        facecolors=(0, 0, 0, 0),   # fully transparent faces
        edgecolor=(0.7, 0.7, 0.7, 0.08),  # faint grid lines
    )

    # ---------- colored changed voxels ----------

    ax.voxels(
        x_grid, y_grid, z_grid,
        filled,
        facecolors=colors,
        edgecolor="black",
        alpha=0.85
    )

    # drone
    ax.scatter(
        drone_position[0],
        drone_position[1],
        drone_position[2],
        color="black",
        s=100
    )

    # lidar ray
    ax.plot(
        [drone_position[0], ray_end[0]],
        [drone_position[1], ray_end[1]],
        [drone_position[2], ray_end[2]],
        color="cyan",
        linewidth=3
    )

    # ray endpoint
    ax.scatter(
        ray_end[0],
        ray_end[1],
        ray_end[2],
        color="yellow",
        s=80
    )

    # labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # equal aspect ratio
    ax.set_box_aspect(voxel_map.shape)
    ax.set_xlim(0, voxel_map.shape[0] * resolution)
    ax.set_ylim(0, voxel_map.shape[1] * resolution)
    ax.set_zlim(0, voxel_map.shape[2] * resolution)

    plt.title(f"Scan Step {SCAN_STEP}")

    plt.show()