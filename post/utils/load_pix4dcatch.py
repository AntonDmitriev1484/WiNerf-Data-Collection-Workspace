import numpy as np
import json
from scipy.spatial.transform import Rotation as R

def load_pix4dcatch(json_path): 
    with open(json_path, "r") as f:
        data = json.load(f)

    captures = data.get("captures", [])
    if not captures:
        raise ValueError("No captures found in JSON")

    coords = np.array([cap["geolocation"]["coordinates"] for cap in captures])

    # Reference (origin) lat/lon/alt
    R_earth = 6378137.0  # meters
    lat0 = np.deg2rad(coords[0, 0])
    lon0 = np.deg2rad(coords[0, 1])
    alt0 = coords[0, 2]

    positions = np.zeros_like(coords)

    # Convert each lat/lon/alt to local ENU (relative to first point)
    for i in range(coords.shape[0]):
        lat = np.deg2rad(coords[i, 0])
        lon = np.deg2rad(coords[i, 1])
        alt = coords[i, 2]

        dlat = lat - lat0
        dlon = lon - lon0

        x = dlon * np.cos(lat0) * R_earth   # East
        y = dlat * R_earth                  # North
        z = alt - alt0                      # Up

        positions[i, :] = [x, y, z]

    # Extract orientations
    orientations = [cap["orientation"]["angles_deg"] for cap in captures] # Yaw pitch roll
    rotations = R.from_euler("xyz", orientations, degrees=True)  # shape (N,3,3)

    # Build poses: 4x4 matrices
    poses = []
    for pos, rot in zip(positions, rotations.as_matrix()):
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        poses.append(T)
    # poses = np.array(poses)

    # Normalize trajectory

    origin = poses[0]
    normalized_poses = []
    positions = []
    for pose in poses:
        normalized_poses.append(np.linalg.inv(origin) @ pose)
        positions.append(normalized_poses[-1][:3,3])
    positions = np.array(positions)

    return normalized_poses