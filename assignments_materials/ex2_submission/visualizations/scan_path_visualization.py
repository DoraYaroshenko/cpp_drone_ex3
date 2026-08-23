import json
from pathlib import Path
import argparse
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Visualize scan path.")
    parser.add_argument("--log", required=True, type=Path, help="Path to drone logs json/jsonl")
    parser.add_argument("--input_map", required=True, type=Path, help="Path to input map npy")
    parser.add_argument("--output_map", required=True, type=Path, help="Path to output map npy")
    parser.add_argument("--resolution", type=float, default=10.0, help="Voxel resolution in cm")
    
    args = parser.parse_args()

    log_path = args.log
    input_map_path = args.input_map
    output_map_path = args.output_map
    resolution = args.resolution


    # ============================================================
    # load files
    # ============================================================

    data = {"movements": [], "scans": [], "voxels": []}
    with open(log_path) as f:
        for line in f:
            obj = json.loads(line)
            data[obj["type"] + "s"].append(obj)
    logs = data

    # input_map_zyx = np.load(input_map_path)
    # output_map_zyx = np.load(output_map_path)
    input_map = np.load(input_map_path)
    output_map = np.load(output_map_path)
    # # convert z,y,x -> x,y,z
    # input_map = np.transpose(input_map_zyx, (2, 1, 0))
    # output_map = np.transpose(output_map_zyx, (2, 1, 0))


    # ============================================================
    # drone path
    # ============================================================

    movements = sorted(
        logs["movements"],
        key=lambda m: m["step_id"]
    )

    path = np.array([
        [
            m["x_cm"],
            m["y_cm"],
            m["z_cm"]
        ]
        for m in movements
    ])

    start_position = path[0]
    end_position = path[-1]


    # ============================================================
    # voxel coloring helper
    # ============================================================

    def build_voxel_data(voxel_map):

        filled = voxel_map != -3

        colors = np.empty(voxel_map.shape, dtype=object)

        for x in range(voxel_map.shape[0]):
            for y in range(voxel_map.shape[1]):
                for z in range(voxel_map.shape[2]):

                    value = voxel_map[x, y, z]

                    # occupied
                    if value == 1:
                        colors[x, y, z] = (
                            1.0, 0.0, 0.0, 0.18
                        )

                    # free
                    elif value == 0:
                        colors[x, y, z] = (
                            0.0, 1.0, 0.0, 0.05
                        )

                    # unreachable
                    elif value == -1:
                        colors[x, y, z] = (
                            0.0, 0.0, 1.0, 0.10
                        )

                    # out of bounds
                    elif value == -2:
                        colors[x, y, z] = (
                            0.5, 0.5, 0.5, 0.03
                        )

                    # unknown
                    else:
                        colors[x, y, z] = (
                            0.0, 0.0, 0.0, 0.0
                        )

        return filled, colors


    input_filled, input_colors = build_voxel_data(input_map)

    output_filled, output_colors = build_voxel_data(output_map)


    # ============================================================
    # figure
    # ============================================================

    fig = plt.figure(figsize=(18, 10))

    ax1 = fig.add_subplot(
        121,
        projection="3d"
    )

    ax2 = fig.add_subplot(
        122,
        projection="3d"
    )


    # ============================================================
    # draw helper
    # ============================================================

    def draw_scene(ax, voxel_map, filled, colors, title):

        # grid coordinates
        x, y, z = np.indices((voxel_map.shape[0]+1, voxel_map.shape[1]+1, voxel_map.shape[2]+1)) * resolution

        # faint grid
        grid = np.ones(voxel_map.shape, dtype=bool)

        ax.voxels(
            x, y, z,
            grid,

            facecolors=(0, 0, 0, 0),

            edgecolor=(
                0.6,
                0.6,
                0.6,
                0.015
            )
        )

        # actual voxels
        ax.voxels(
            x, y, z,
            filled,

            facecolors=colors,

            edgecolor=None
        )

        # drone path
        ax.plot(
            path[:, 0],
            path[:, 1],
            path[:, 2],

            linewidth=1.2,

            color="black",

            alpha=0.5
        )

        # start point
        ax.scatter(
            start_position[0],
            start_position[1],
            start_position[2],

            s=180,

            color="lime",

            label="start"
        )

        # end point
        ax.scatter(
            end_position[0],
            end_position[1],
            end_position[2],

            s=180,

            color="red",

            label="end"
        )

        # directional arrows
        step_stride = max(1, len(path) // 30)

        for i in range(0, len(path) - 1, step_stride):

            p0 = path[i]
            p1 = path[i + 1]

            direction = p1 - p0

            ax.quiver(
                p0[0],
                p0[1],
                p0[2],

                direction[0],
                direction[1],
                direction[2],

                length=1.0,

                normalize=False,

                color="black",

                linewidth=1.2,

                arrow_length_ratio=0.3
            )

        # labels
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.set_title(title)

        ax.set_box_aspect(voxel_map.shape)

        ax.view_init(
            elev=28,
            azim=-58
        )


    # ============================================================
    # draw both scenes
    # ============================================================

    draw_scene(
        ax1,
        input_map,
        input_filled,
        input_colors,
        "Input Map + Drone Path"
    )

    draw_scene(
        ax2,
        output_map,
        output_filled,
        output_colors,
        "Output Map + Drone Path"
    )


    # ============================================================
    # legend
    # ============================================================

    handles, labels = ax1.get_legend_handles_labels()

    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2
    )


    plt.tight_layout()

    plt.show()