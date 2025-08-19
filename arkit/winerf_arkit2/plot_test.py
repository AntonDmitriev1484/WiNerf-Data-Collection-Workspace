import json
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Rotation helpers ---
def rotation_matrix_from_ypr(yaw, pitch, roll, degrees=True):
    """Return 3x3 rotation matrix from yaw, pitch, roll (Z-Y-X convention)."""
    if degrees:
        yaw, pitch, roll = np.deg2rad([yaw, pitch, roll])
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    # Rotation order: Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[cp, 0, sp],
                   [ 0, 1, 0],
                   [-sp, 0, cp]])
    Rx = np.array([[1,  0,  0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    return Rz @ Ry @ Rx


def draw_axes(ax, origin, R, length=1.0):
    """Draw XYZ axes at given origin with rotation matrix R."""
    x_axis = R[:, 0] * length
    y_axis = R[:, 1] * length
    z_axis = R[:, 2] * length

    ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)


# --- Main parsing & plotting ---
def visualize_captures(json_path, outdir="out", length=2.0):
    with open(json_path, "r") as f:
        data = json.load(f)

    captures = data.get("captures", [])

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')

    stride = 10

    for idx, cap in enumerate(captures):
        coords = cap["geolocation"]["coordinates"]  # [lat, lon, alt] or [x,y,z]
        yaw, pitch, roll = cap["orientation"]["angles_deg"]

        origin = np.array(coords)
        R = rotation_matrix_from_ypr(yaw, pitch, roll)

        # if idx % stride == 0:
            # draw_axes(ax, origin, R, length=length)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Capture {idx} Orientation")

        # Center view around origin
        ax.set_xlim(origin[0] - 5, origin[0] + 5)
        ax.set_ylim(origin[1] - 5, origin[1] + 5)
        ax.set_zlim(origin[2] - 5, origin[2] + 5)

        # Fixed top-down view (XY plane)
        ax.view_init(elev=90, azim=-90)

    # Save headless PNG
    outpath = f"./capture_{idx}.png"
    fig.savefig(outpath, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {outpath}")


if __name__ == "__main__":
    visualize_captures("./input_cameras.json", outdir="results")
