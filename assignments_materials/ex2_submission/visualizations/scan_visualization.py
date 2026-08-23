import json
import argparse
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


# ---------- voxel helpers ----------

def voxel_trace(mask, color, name, resolution, opacity=0.5):
    x, y, z = np.where(mask)
    
    # offset to center of voxel
    x = (x + 0.5) * resolution
    y = (y + 0.5) * resolution
    z = (z + 0.5) * resolution

    return go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode="markers",
        name=name,
        marker=dict(
            size=4,
            color=color,
            opacity=opacity,
            symbol="square"
        ),
        showlegend=True
    )


def build_map_traces(voxel_map, resolution, prefix=""):
    occupied = voxel_map == 1
    free = voxel_map == 0
    unreachable = voxel_map == -1
    out_of_bounds = voxel_map == -2
    unknown = voxel_map == -3

    traces = [
        voxel_trace(occupied, "red", f"{prefix}occupied", resolution, 0.35),
        voxel_trace(unreachable, "blue", f"{prefix}unreachable", resolution, 0.35),
        voxel_trace(out_of_bounds, "gray", f"{prefix}out_of_bounds", resolution, 0.15),
        voxel_trace(unknown, "purple", f"{prefix}unknown", resolution, 0.00),
        voxel_trace(free, "green", f"{prefix}free", resolution, 0.08),
    ]

    return traces


def drone_trace(position, color="black", scene=None):
    tr = go.Scatter3d(
        x=[position[0]],
        y=[position[1]],
        z=[position[2]],
        mode="markers",
        marker=dict(
            size=8,
            color=color,
            symbol="diamond"
        ),
        name="drone",
        showlegend=False
    )

    if scene is not None:
        tr.update(scene=scene)

    return tr


def lidar_ray_trace(origin, direction, length, color="orange", scene=None):
    end = (
        origin[0] + direction[0] * length,
        origin[1] + direction[1] * length,
        origin[2] + direction[2] * length,
    )

    tr = go.Scatter3d(
        x=[origin[0], end[0]],
        y=[origin[1], end[1]],
        z=[origin[2], end[2]],
        mode="lines",
        line=dict(
            color=color,
            width=4
        ),
        name="lidar_ray",
        showlegend=False
    )

    if scene is not None:
        tr.update(scene=scene)

    return tr


# ---------- angle helpers ----------

def orientation_to_direction(h_deg, v_deg):
    h = np.deg2rad(h_deg)
    v = np.deg2rad(v_deg)

    dx = np.cos(v) * np.cos(h)
    dy = np.cos(v) * np.sin(h)
    dz = np.sin(v)

    return (dx, dy, dz)


# ---------- simulation ----------

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize scan.")
    parser.add_argument("--log", required=True, type=Path, help="Path to drone logs json/jsonl")
    parser.add_argument("--input_map", required=True, type=Path, help="Path to input map npy")
    parser.add_argument("--output_map", required=True, type=Path, help="Path to output map npy")
    # parser.add_argument("--capabilities", required=True, type=Path, help="Path to drone capabilities json")
    parser.add_argument("--resolution", type=float, default=10.0, help="Voxel resolution in cm")

    args = parser.parse_args()

    log_path = args.log
    input_map_path = args.input_map
    output_map_path = args.output_map
    # capabilities_path = args.capabilities
    resolution = args.resolution

    # ---------- load configs ----------

    # with open(capabilities_path) as f:
    #     capabilities = json.load(f)

    # lidar_max_range = capabilities["lidar"]["beam_length_max"]
    lidar_max_range = 400
    # ---------- load logs ----------
    data = {"movements": [], "scans": [], "voxels": []}
    with open(log_path) as f:
        for line in f:
            obj = json.loads(line)
            data[obj["type"] + "s"].append(obj)

    movement_log = data["movements"]
    scan_log = data["scans"]
    voxel_log = data["voxels"]

    movement_by_step = {
        entry["step_id"]: entry
        for entry in movement_log
    }

    scan_by_step = {
        entry["step_id"]: entry
        for entry in scan_log
    }

    # ---------- load maps ----------

    input_map = np.load(input_map_path)
    output_map = np.load(output_map_path)

    assert input_map.shape == output_map.shape
    assert len(input_map.shape) == 3


    # evolving map starts unknown
    evolving_map = np.full_like(output_map, -3)

    # immediately mark out-of-bounds
    evolving_map[output_map == -2] = -2

    # ---------- voxel updates by step ----------

    voxel_updates_by_step = {}

    for entry in voxel_log:
        sid = entry["step_id"]

        voxel_updates_by_step.setdefault(sid, []).append(entry)

    all_step_ids = sorted(set(
        list(movement_by_step.keys()) +
        list(scan_by_step.keys()) +
        list(voxel_updates_by_step.keys())
    ))

    # ---------- figure ----------

    fig = make_subplots(
        rows=1,
        cols=2,
        specs=[[{"type": "scene"}, {"type": "scene"}]],
        subplot_titles=("Ground Truth", "Scanning Progress")
    )

    # ---------- initial traces ----------

    input_traces = build_map_traces(input_map, resolution, "input_")
    process_traces = build_map_traces(evolving_map, resolution, "process_")

    for tr in input_traces:
        fig.add_trace(tr, row=1, col=1)

    for tr in process_traces:
        fig.add_trace(tr, row=1, col=2)

    # ---------- initial drone ----------

    current_position = (0, 0, 0)
    current_h = 0
    current_v = 0

    if movement_log:

        first = movement_log[0]

        current_position = (
            first["x_cm"],
            first["y_cm"],
            first["z_cm"]
        )

        current_h = first["h_angle_deg"]
        current_v = first["v_angle_deg"]

    fig.add_trace(
        drone_trace(current_position, scene="scene"),
        row=1,
        col=1
    )

    fig.add_trace(
        drone_trace(current_position, scene="scene2"),
        row=1,
        col=2
    )

    # ---------- animation frames ----------

    frames = []

    for sid in all_step_ids:

        # movement update
        if sid in movement_by_step:

            movement = movement_by_step[sid]

            current_position = (
                movement["x_cm"],
                movement["y_cm"],
                movement["z_cm"]
            )

            current_h = movement["h_angle_deg"]
            current_v = movement["v_angle_deg"]

        # voxel updates
        if sid in voxel_updates_by_step:

            for v in voxel_updates_by_step[sid]:

                evolving_map[
                    v["x"],
                    v["y"],
                    v["z"]
                ] = v["value"]

        process_traces = build_map_traces(evolving_map, resolution, "process_")

        frame_data = []

        # ---------- left scene ----------

        for tr in build_map_traces(input_map, resolution, "input_"):
            tr.update(scene="scene")
            frame_data.append(tr)

        frame_data.append(
            drone_trace(current_position, scene="scene")
        )

        # ---------- right scene ----------

        for tr in process_traces:
            tr.update(scene="scene2")
            frame_data.append(tr)

        frame_data.append(
            drone_trace(current_position, scene="scene2")
        )

        # lidar visualization
        if sid in scan_by_step:

            scan = scan_by_step[sid]

            absolute_h = (
                current_h +
                scan["relative_h_angle_deg"]
            )

            absolute_v = (
                current_v +
                scan["relative_v_angle_deg"]
            )

            direction = orientation_to_direction(
                absolute_h,
                absolute_v
            )

            frame_data.append(
                lidar_ray_trace(
                    current_position,
                    direction,
                    lidar_max_range,
                    scene="scene2"
                )
            )

        frames.append(
            go.Frame(
                data=frame_data,
                name=str(sid)
            )
        )

    fig.frames = frames

    # ---------- layout ----------

    common_scene = dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",

        aspectmode="data",

        # fixed camera
        camera=dict(
            eye=dict(x=1.6, y=1.6, z=1.2)
        ),

        # preserve orientation during animation
        uirevision="fixed"
    )

    fig.update_layout(

        title="Drone Scan Simulation",

        scene=common_scene,
        scene2=common_scene,

        height=900,

        showlegend=False,

        updatemenus=[
            {
                "type": "buttons",
                "direction": "left",
                "x": 0.1,
                "y": 1.15,
                "showactive": False,

                "buttons": [
                    {
                        "label": "Play",
                        "method": "animate",
                        "args": [
                            None,
                            {
                                "frame": {
                                    "duration": 200,
                                    "redraw": True
                                },
                                "transition": {
                                    "duration": 0
                                },
                                "fromcurrent": True,
                                "mode": "immediate"
                            }
                        ]
                    },
                    {
                        "label": "Pause",
                        "method": "animate",
                        "args": [
                            [None],
                            {
                                "frame": {
                                    "duration": 0,
                                    "redraw": False
                                },
                                "mode": "immediate"
                            }
                        ]
                    }
                ]
            }
        ],

        sliders=[
            {
                "active": 0,

                "steps": [
                    {
                        "label": str(sid),

                        "method": "animate",

                        "args": [
                            [str(sid)],
                            {
                                "frame": {
                                    "duration": 0,
                                    "redraw": True
                                },
                                "transition": {
                                    "duration": 0
                                },
                                "mode": "immediate"
                            }
                        ]
                    }

                    for sid in all_step_ids
                ]
            }
        ]
    )

    fig.show()