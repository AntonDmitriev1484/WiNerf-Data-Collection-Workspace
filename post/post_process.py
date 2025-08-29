
from pathlib import Path
import pkgutil
import importlib
import inspect
import os
import json
import csv
import yaml
import argparse

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.optimize import minimize
from types import SimpleNamespace

import shutil
import math
import copy

from utils.load_rostypes import *
from utils.ros_msg_handlers import *
from utils.apriltag import *
from utils.math_utils import *
from utils.load_pix4dcatch import *


import matplotlib.pyplot as plt
matplotlib.use('TkAgg')

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import pickle
# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10

# python3 post_process.py -t winerf_prelim2 -c cam_target_daslab --crop_start 186 --override_april_start='[0,0.25,0.2535]' --in_tx_location='[0,3,0.57]' --plot_world True

parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--cam_calibration_file", "-c", type=str)
parser.add_argument("--crop_start", type=float) # From now on this will be relative time from start of recording instead of an absolute timestamp
parser.add_argument("--override_april_start", type=str )
parser.add_argument("--start_position_cam", type=str ) # start position of world origin in cam1 frame, at alignment point.
parser.add_argument("--z_rot_adjust_deg", default=0, type=float)
parser.add_argument("--use_arkit", default=False, type=bool)
parser.add_argument("--in_tx_location", type=str )
parser.add_argument("--plot_world", default=False, type=bool) # Generate a plot of router positions and aoa vectors in world frame when done?

args = parser.parse_args()

outpath = f'./out/{args.trial_name}_post'

out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_slam = f'{outpath}/slam'

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_slam, exist_ok=True)

in_router_data = f'../router/{args.trial_name}.npz'
in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_arkit = f'../arkit/{args.trial_name}/input_cameras.json'
in_kalibr = f"../kalibr/camimu_out/{args.cam_calibration_file}-camchain-imucam.yaml"
in_tx = f"../world/{args.in_tx_location}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')


if args.use_arkit:
    slam_kf_data = None
    slam_data = load_pix4dcatch(in_arkit) # Loads data and performs pre-processing transforms for us to use.
else:
    slam_kf_data = np.loadtxt(in_slam_kf)
    slam_kf_data[:,0] *= 1e-9
    slam_data = np.loadtxt(in_slam)
    slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'
    
router_data = np.load(in_router_data)
print(f" {router_data.files=}")

t_router = router_data['timestamps']
csi_data = router_data['csi_matrix']
aoa_rx_frame = router_data['aoa_matrix']
signal_strength = router_data['strength']

# for i in range(len(aoa_rx_frame)):
#     for j in range(3):
#         y = aoa_rx_frame[i, j, 1]
#         aoa_rx_frame[i, j, 1] = aoa_rx_frame[i, j, 0]
#         aoa_rx_frame[i, j, 0] = y

all_data = []
gt_standalone = []

rostypes = load_rostypes()
print(rostypes)

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=rostypes) as reader:
    # Filter for messages within bag timestamp range.
    START_ = reader.start_time * 1e-9
    END_ = reader.end_time * 1e-9
    # args.crop_start += START_ # TODO add back in later
# START = START_ + 138
# END = START_ + 158

START = START_
END = END_

print(f"ROS duration {START} - {END}")
print(f"Data start {START} cropped to {args.crop_start}")

# winerf_prelim2
# ROS duration 1755120873.7234828 - 1755121110.4828684
# Data start 1755120873.7234828 cropped to 1755121059.7234828


def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


### Define all coordinate transforms

# body = rx
# cam1 = sbody
# cam1(t=0) = sorigin
Transforms = SimpleNamespace()


Transforms.T_cam1_to_rx = np.eye(4)

# t_cam1_to_rx_in_cam1 = np.array([-0.1475,0.01, -0.081])
# R_cam1_to_rx = np.array([[0,0,1],
#                         [-1,0,0],
#                         [0,-1,0]])
# t_rx_to_cam1_in_rx =  np.array([0.1475,0.01, -0.081])

R_cam1_to_rx = np.array([[0,0,-1],
                        [-1,0,0],
                        [0,-1,0]])
t_rx_to_cam1_in_rx =  np.array([-0.1475,0.01, -0.081])
# Proper rotation from MUSIC frame to camera


Transforms.T_cam1_to_rx = np.eye(4)
Transforms.T_cam1_to_rx[:3,:3] = R_cam1_to_rx
Transforms.T_cam1_to_rx[:3,3]= t_rx_to_cam1_in_rx

# Transforms.T_cam1_to_rx[:3,:3] = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
Transforms.T_body_to_cam1 = np.linalg.inv(Transforms.T_cam1_to_rx)

# Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)
# Transforms = extract_apriltag_pose_PnP(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

if args.start_position_cam is None:
    print(" Need to set manual starting point for this dataset collection!")
    print(" Sorry :(")
    exit()

t_align = args.crop_start -1 # Assume alignment point is the same as what we want to set
T_world_to_body_at_t_align = np.eye(4)
# cam1_z = 0.2535

#
t_cam1_to_world_in_cam1 = np.array(json.loads(args.start_position_cam))

# Assume camera frame Z is always aligned with world Y
R_world_to_cam1 = np.array([[1,0,0], 
                            [0,0,-1], 
                            [0,1,0]])

# Sometimes you need to apply a manual rotation about the Z-axis so that the trajectory can work properly.
Transforms.T_adjust_in_world = np.eye(4)
if args.z_rot_adjust_deg != 0.0:
    print("ADJUSTING")
    theta = np.deg2rad(args.z_rot_adjust_deg)
    Transforms.T_adjust_in_world[:3,:3] = np.array(
        [[np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]])

T_world_to_cam1_talign = np.eye(4)
T_world_to_cam1_talign[:3,:3] = R_world_to_cam1
T_world_to_cam1_talign[:3,3] = t_cam1_to_world_in_cam1

# T_world_to_body_at_t_align[:3, 3] = np.array([0.75, 3, cam1_z])
# T_world_to_body_at_t_align[:3,:3] = R_prior

# Fetch the closest SLAM pose to our crop start
idx = np.argmin(np.abs(slam_data[:,0] - t_align))
print(f"{idx=}")

T_cam1_to_sorigin_talign = slam_quat_to_HTM(slam_data[idx])

# Transforms.T_world_to_sorigin =  T_world_to_cam1_talign @ np.linalg.inv(T_sorigin_to_sbody_at_t_align)
Transforms.T_world_to_cam1_talign = T_world_to_cam1_talign
Transforms.T_world_to_sorigin = T_cam1_to_sorigin_talign @ T_world_to_cam1_talign @ Transforms.T_adjust_in_world

print(f"{T_world_to_body_at_t_align=}")
print(f"{T_cam1_to_sorigin_talign=}")
print(f"{Transforms.T_world_to_sorigin=}")


def get_T_world_to_body(slam_pose): # A function because I re-use this a lot
    T_cam1_to_sorigin = slam_pose
    T_world_to_body = Transforms.T_cam1_to_rx @ np.linalg.inv(T_cam1_to_sorigin) @ Transforms.T_world_to_sorigin
    return T_world_to_body

Transforms.origin = np.eye(4)

### Write SLAM camera trajectory
slam_poses_slam_frame = [] # This is a list of T_sorigin_to_sbody
slam_pose_counter = 0

body_poses_world_frame = []

for i in range(slam_data.shape[0]-1):
    T_sorigin_to_sbody = slam_quat_to_HTM(slam_data[i,:])
    body_poses_world_frame.append(get_T_world_to_body(T_sorigin_to_sbody))
    slam_poses_slam_frame.append( [slam_data[i,0]] + list(T_sorigin_to_sbody.flatten()) )
    slam_pose_counter += 1


# Using the SLAM trajectory (body pose in the slam frame), and known AoA timestamps
# Compute an interpolated SLAM trajectory of the body pose in the world frame,
# s.t. each pose is timestamp aligned to each AoA measurement.

# Check timestamps
# fig, ax = plt.subplots()
# # Plot each array on a horizontal line
# ax.scatter(slam_data[:,0], np.zeros_like(slam_data[:,0]), c="blue", label="all_slam", alpha=0.7)
# ax.scatter(t_router[:], np.ones_like(t_router[:]), c="red", label="t_router", alpha=0.7)
# ax.axvline(x=START_+args.crop_start)

# # Beautify
# ax.set_yticks([0, 1])
# ax.set_xlabel("Value")
# ax.set_title("1D Scatter Plot of Two Arrays")
# ax.legend()

# plt.show()


N_POINTS = 100
body_poses_world_frame = [] #Poses of body in world frame, interpolated to match AoA measurements.
for i in range(t_router.shape[0]):

    # Get the closest SLAM measurements to the router timestamp
    tdiffs = np.abs(slam_data[:,0] - t_router[i])
    slam_idx1 = np.argmin(tdiffs)
    tdiffs[slam_idx1] = np.inf
    slam_idx2 = np.argmin(tdiffs)
    istart, iend = sorted([slam_idx1, slam_idx2]) # Make sure indices are ascending

    # Poses we're interpolating for are in slam frame
    current_pose = slam_quat_to_HTM(slam_data[istart, :])
    next_pose = slam_quat_to_HTM(slam_data[iend, :])

    # Now interpolate between these two poses
    interp_interval = [slam_data[istart,0], slam_data[iend, 0]]
    interp_timestamps = np.linspace(slam_data[istart,0], slam_data[iend, 0], N_POINTS)

    # Use Slerp to interpolate on SO(3) rotations
    interp_rots = R.from_matrix([current_pose[:3, :3], next_pose[:3, :3]])
    slurpy = Slerp(interp_interval, interp_rots)
    interpolated_rotations = slurpy(interp_timestamps)

    # Use linspace to interpolate on R3 positions
    interpolated_positions = np.linspace(current_pose[:3, 3], next_pose[:3, 3], N_POINTS)

    # Get the closest interpolation timestamp to the router timestamp,
    # and map that interpolated pose to the measurement
    idx_match = np.argmin(np.abs(interp_timestamps - t_router[i]))

    interpolated_slam_frame_pose = np.eye(4)
    interpolated_slam_frame_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
    interpolated_slam_frame_pose[:3, 3] = interpolated_positions[idx_match]

    # Now convert interpolated pose to world frame
    world_frame_pose = get_T_world_to_body(interpolated_slam_frame_pose)
    # Interpolating in SLAM frame, then converting to world, helps avoid LHS coord system issue.

    body_poses_world_frame.append(world_frame_pose)


### Now convert all AoA vectors to the world frame

all_aoa_vectors_world_frame = []
all_aoa_vectors_world_frame_rotation = []

#


tx_loc = np.array(json.load(open(in_tx, 'r'))["position"])
SYNTHETIC_AOA = False # Just for testing that my transforms are correct.
import random
if SYNTHETIC_AOA:
    # In a circle, we expect the AoA to always be pointing inwards, this would be along the +y axis of the rx fraeme
    aoa_rx_frame[:] = np.array([1,0,0])
    
    for i, pose in enumerate(body_poses_world_frame):
        T_world_to_rx = pose
        vec = tx_loc - np.linalg.inv(T_world_to_rx)[:3,3]
        synth_aoa_world_frame = vec / np.linalg.norm(vec)
        aoa_rx_frame[i, :] = np.linalg.inv(T_world_to_rx[:3,:3]) @ synth_aoa_world_frame

    # Assume AoA vectors at 45 deg.
    # deg_from_x_axis = 75
    # # # Are we positivee, or negative 75 deg from the x-axis?

    # aoa_rx_frame[:] = np.array([np.cos(np.deg2rad(deg_from_x_axis)), np.sin(np.deg2rad(deg_from_x_axis)), 0])
    # for i in range(aoa_rx_frame.shape[0]):
    #     for j in range(3):
    #         aoa_rx_frame[i,j,1] *= random.choice([1, -1])

all_aoa_vectors_world_frame_rotation_reflected = []

for i in range(t_router.shape[0]):

    aoa_vectors_rx_frame = aoa_rx_frame[i, :] # Should be an array of N paths x 3
    n_paths = aoa_vectors_rx_frame.shape[0]


    aoa_vectors_world_frame = np.empty_like(aoa_vectors_rx_frame)
    aoa_vectors_world_frame_reflected = np.empty_like(aoa_vectors_rx_frame)
    
    # Using the known pose of the body in the world frame, at time t
    # Map each path at time t into the world frame.

    T_world_to_rx = body_poses_world_frame[i]
    T_rx_to_world = np.linalg.inv(T_world_to_rx)

    # # This computation takes the 'tip' of the AoA unit vector in rx frame coordinates
    # # And maps it to the 'tip' of the AoA unit vector in world frame coordinates.
    # # This means that this will return points w.r.t world origin, and not w.r.t rx frame origin in world frame.
    # for path_idx in range(n_paths):
    #     aoa_vectors_world_frame[path_idx,:] = (T_rx_to_world @ np.append(aoa_vectors_rx_frame[path_idx, :], 1))[:3]
    # all_aoa_vectors_world_frame.append(aoa_vectors_world_frame)

    # This computation rotates the AoA unit vector from rx frame coordinates into world frame coordinates.
    # This means that the tip of this unit vector will be w.r.t 0,0,0, but its rotation will be
    # consistent with the world axes.
    aoa_vectors_world_frame_rotation = np.empty_like(aoa_vectors_rx_frame)
    aoa_vectors_world_frame_rotation_reflected = np.empty_like(aoa_vectors_rx_frame)
    for path_idx in range(n_paths):
        v_rx = aoa_vectors_rx_frame[path_idx, :]
        v_world = T_rx_to_world[:3,:3] @ aoa_vectors_rx_frame[path_idx, :]
        aoa_vectors_world_frame_rotation[path_idx, :] = T_rx_to_world[:3,:3] @ np.array([[1,0,0],[0,-1,0],[0,0,1]]) @ aoa_vectors_rx_frame[path_idx, :]
        aoa_vectors_world_frame_rotation_reflected[path_idx, :] = T_rx_to_world[:3,:3] @ np.array([[-1,0,0],[0,-1,0],[0,0,1]]) @ aoa_vectors_rx_frame[path_idx, :]
        # aoa_vectors_world_frame_rotation[path_idx, :] = T_rx_to_world[:3,:3] @ aoa_vectors_rx_frame[path_idx, :]
        # aoa_vectors_world_frame_rotation_reflected[path_idx, :] = T_rx_to_world[:3,:3] @ aoa_vectors_rx_frame[path_idx, :]

    all_aoa_vectors_world_frame_rotation_reflected.append(aoa_vectors_world_frame_rotation_reflected)
    all_aoa_vectors_world_frame_rotation.append(aoa_vectors_world_frame_rotation)
    # These will provide the RPY of the vector along world frame axes, as measured at the receiver's origin.

valid_timestamp_idx = list(np.where((t_router > args.crop_start) & (t_router < END))[0])

# Apply timestamp filtering here.
positions_world = []
aoa_vectors_world = []
aoa_vectors_world_r = []


i=0

body_poses_world_frame_ = []
for body_pose, aoa_vector, aoa_vector_r in zip(body_poses_world_frame, all_aoa_vectors_world_frame_rotation, all_aoa_vectors_world_frame_rotation_reflected):
    if i in valid_timestamp_idx:
        H = np.linalg.inv(body_pose)
        positions_world.append(H[:3,3].flatten())
        aoa_vectors_world.append(aoa_vector)
        aoa_vectors_world_r.append(aoa_vector_r)
        body_poses_world_frame_.append(body_pose)
    i+=1
body_poses_world_frame = body_poses_world_frame_

print(positions_world[:5])
np.savez(
        outpath+"/roomba_data_world.npz", 
        timestamps = router_data['timestamps'], 
        csi_data = router_data['csi_matrix'],
        aoa_matrix = router_data['aoa_matrix'],
        strength = router_data['strength'],
        aoa_matrix_world=aoa_vectors_world, 
        positions_world=positions_world
        )


### Copy all world information: transforms, anchors, apriltags, to output
with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


# Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
print("Checking frequency of real data")
print(f" Measured SLAM frequency {len(slam_data) / (END-START)}")
print(f" Measured CSI frequency {t_router.shape[0] / (t_router[-1]-t_router[0])}")


json.dump(args.__dict__, open(outpath+"/meta.json", 'w'), cls=NumpyEncoder, indent=1)

# Plot results

body_orientation_stride = 100
aoa_vector_stride = 10

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

positions_world = np.array(positions_world)
aoa_vectors_world = np.array(aoa_vectors_world)
ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2], label='Trajectory', color='blue')
ax.scatter(*positions_world[0], color='green', label='Start')
ax.scatter(*positions_world[-1], color='red', label='End')

tx_loc = np.array(json.load(open(in_tx, 'r'))["position"])

ax.scatter(*tx_loc, color = 'purple', s=150)


def draw_axes(ax, T, length=0.1, is_tx=False):
    """Draw coordinate axes from transformation matrix T."""

    H = np.linalg.inv(T)
    origin = (H @ np.array([0,0,0,1]))[:3]
    x_axis = (H @ np.array([1,0,0,1]))[:3]
    y_axis = (H @ np.array([0,1,0,1]))[:3]
    z_axis = (H @ np.array([0,0,1,1]))[:3]

    if is_tx:
        ax.quiver(*origin, *(x_axis-origin) * length, color='pink')
        ax.quiver(*origin, *(y_axis-origin) * length, color='pink')
        ax.quiver(*origin, *(z_axis-origin) * length, color='pink')
    else:
        ax.quiver(*origin, *(x_axis-origin) * length, color='r')
        ax.quiver(*origin, *(y_axis-origin) * length, color='g')
        ax.quiver(*origin, *(z_axis-origin) * length, color='b')



if body_orientation_stride > 0:
    for i in range(0, len(body_poses_world_frame), body_orientation_stride):
        draw_axes(ax, body_poses_world_frame[i], length=0.1)

max_strength_idx = np.argmax(signal_strength)
print(max_strength_idx)
min_strength = 1e-5
maxi, maxj = np.unravel_index(max_strength_idx, signal_strength.shape)
max_strength = signal_strength[maxi, maxj] / 1e4

if aoa_vector_stride > 0:
    length = 0.2
    n_vectors = 1 # Plot first N paths
    for i in range(0, len(positions_world), aoa_vector_stride):
        for j in range(n_vectors):
            strength = signal_strength[i,j]
            origin = positions_world[i]
            tip = aoa_vectors_world[i][j,:]
            scale = (strength - min_strength)/max_strength
            ax.quiver(*origin, *tip, color='purple', length=length )

            tip2 = aoa_vectors_world_r[i][j,:]
            scale = (strength - min_strength)/max_strength
            ax.quiver(*origin, *tip2, color='purple', length=length )

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Trajectory and Static Coordinate Frames")
ax.view_init(elev=20, azim=45)
ax.legend()
ax.set_xlim(-1, 3)
ax.set_ylim(0,4)
ax.set_zlim(0,4)
plt.show()

ax.view_init(elev=90, azim=-90)  # elev=90 gives top-down
# Save to PNG with controlled resolution
plt.savefig(outpath+"trial_viz.jpg", dpi=600, bbox_inches='tight')
pickle.dump(fig, open(outpath+"/trial_viz.pickle", 'wb'))

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, '__dict__'):
            return vars(obj)
        return super().default(obj)

