import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

def load_tum_trajectory(filepath):
    """
    Load a TUM trajectory file: timestamp tx ty tz qx qy qz qw
    Returns:
        times: np.array of timestamps
        poses: np.array of translations (N x 3)
    """
    times = []
    poses = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip().startswith("#") or line.strip() == "":
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            try:
                t, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
                times.append(t)
                poses.append([tx, ty, tz])
            except ValueError:
                continue
    return np.array(times), np.array(poses)


def plot_trajectory_3d(poses, title="3D Trajectory"):
    """
    Plot trajectory positions in an interactive 3D plot.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], label="Trajectory", color="blue")
    ax.scatter(poses[0, 0], poses[0, 1], poses[0, 2], color="green", s=50, label="Start")
    ax.scatter(poses[-1, 0], poses[-1, 1], poses[-1, 2], color="red", s=50, label="End")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.legend()

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot 3D trajectory from a TUM trajectory file.")
    parser.add_argument("filename", help="Trajectory file located inside 'out/' (e.g. traj.txt)")
    args = parser.parse_args()

    tum_file = os.path.join("out", args.filename)
    if not os.path.exists(tum_file):
        print(f"Error: File {tum_file} does not exist.")
        return

    times, poses = load_tum_trajectory(tum_file)
    plot_trajectory_3d(poses, title=f"3D Trajectory from {args.filename}")


if __name__ == "__main__":
    main()

