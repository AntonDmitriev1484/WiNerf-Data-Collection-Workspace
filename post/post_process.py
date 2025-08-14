
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


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import pickle
# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10

# python3 post_process.py -t winerf_prelim2 -c cam_target_daslab --crop_start 186 --override_april_start='[0,0.25,0.2535]' --in_tx_location='[0,3,0.57]' --plot_world True

parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--cam_calibration_file", "-c", type=str)
parser.add_argument("--crop_start", type=float) # From now on this will be relative time instead of an absolute timestamp
parser.add_argument("--override_april_start", type=str )
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
in_kalibr = f"../kalibr/camimu_out/{args.cam_calibration_file}-camchain-imucam.yaml"
in_tx = f"../world/{args.in_tx_location}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

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


# Need to maintain another array that we can buffer data to before dumping one sensor per csv
topic_to_processing = {
                  '/camera/camera/imu': (proc_imu, []),
                  '/camera/camera/infra1/image_rect_raw': (proc_infra1_frame, []),
                  '/camera/camera/infra2/image_rect_raw': (proc_infra2_frame, []),
}

all_data = []
dataset_topics = [ k for k,v in topic_to_processing.items()]
gt_standalone = []

rostypes = load_rostypes()
print(rostypes)

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=rostypes) as reader:
    connections = [x for x in reader.connections if x.topic in dataset_topics]
    for connection, timestamp, rawdata in reader.messages(connections=connections):

        try:
            msg = reader.deserialize(rawdata, connection.msgtype)
            proc, arr_ref = topic_to_processing[connection.topic]
            proc(msg, arr_ref)

        except Exception as e:
            print( e)
            continue  # optionally log here

# Processors functions have now buffered their individual topics into arr_ref
# This is useful for writing the same datastream to multiple files.
# Then, lastly, we can create all.json using the buffered measurements.



# Filter for messages within bag timestamp range.
START = reader.start_time * 1e-9
END = reader.end_time * 1e-9
args.crop_start += START

print(f"ROS duration {START} - {END}")
print(f"Data start {START} cropped to {args.crop_start}")

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
Transforms.T_cam1_to_rx[:3, 3] = np.array([0,-0.089, -0.1525])
Transforms.T_cam1_to_rx[:3,:3] = np.linalg.inv(np.array([[0,0,1], [-1,0,0], [0,-1,0]]))
# Transforms.T_cam1_to_rx[:3,:3] = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
Transforms.T_body_to_cam1 = np.linalg.inv(Transforms.T_cam1_to_rx)

infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]
# Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)
# Transforms = extract_apriltag_pose_PnP(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

if args.override_april_start is None:
    print(" Need to set manual starting point for this dataset collection!")
    print(" Sorry :(")
    exit()

# Compute initial pose estimate using optimizer.
# Super hacky solution. Basically just forces trajectory to be in XY plane at a Z level with the origin.
# We can do this because it's a roomba moving on flat ground.
T_prior = np.eye(4)
R_prior = np.array([[1, 0 ,0 ],
                    [0, 0, 1],
                    [0, -1, 0]], dtype=np.float64)
T_prior[:3,:3] = R_prior
T_prior[:3,3] = np.array(json.loads(args.override_april_start))
Transforms.T_world_to_sorigin = np.eye(4)
Transforms.T_world_to_sorigin[:3, 3] = np.array(json.loads(args.override_april_start))
Transforms.T_world_to_sorigin[:3,:3] = R_prior
best_Z = Transforms.T_world_to_sorigin[2,3]

# def rotate_about_world_x(T, theta):
#     # Rotate to world frame, then rotate by the adjustment along world frame x axis.
#     R_about_world_x = np.array([[1, 0, 0, 0], 
#                                 [0, np.cos(theta), -np.sin(theta), 0],
#                                 [0, np.sin(theta), np.cos(theta), 0],
#                                 [0,0,0,1]], dtype=np.float64)
#     T_ = T.copy()
#     T_ =  R_about_world_x @ T
#     return T_

# def minimize_for_world_pose(slam_data, best_Z, T_world_to_sorigin, Transforms):


#     def get_T_world_to_body(T_sorigin_to_sbody): # A function because I re-use this a lot
#         T_world_to_body = (
#                         Transforms.T_world_to_sorigin 
#                         @ T_sorigin_to_sbody 
#                         @ Transforms.T_cam1_to_rx
#         )
#         return T_world_to_body

#     body_poses_world_frame = []
#     zs = []
#     for i in range(slam_data.shape[0]-1):

#         T_sorigin_to_sbody = slam_quat_to_HTM(slam_data[i,:])
#         T_world_to_body = get_T_world_to_body(T_sorigin_to_sbody)
#         body_poses_world_frame.append( T_world_to_body )
#         zs.append(T_world_to_body[2,3])

#     # The best (lowest) score has the lowest total error in each poses Z-coordinate
#     # score = sum([ abs( best_Z - T_world_to_body[2,3]) ** 2 for T_world_to_body in body_poses_world_frame])
#     score = np.var(np.abs(np.array(zs)-best_Z))

#     return score

# def distance_to_XY_plane_loss(theta):
#     print(f" Trying theta: {theta * 180/np.pi}")

#     T_world_to_sorigin = np.eye(4)
#     T_world_to_sorigin[:3,:3] = R_prior # Im pretty sure this is the right order for python
#     T_world_to_sorigin[:3,3] = Transforms.T_world_to_sorigin[:3, 3]

#     T_world_to_sorigin = rotate_about_world_x(T_world_to_sorigin, float(theta[0]))

#     # Make sure you deep copy slam data every time
#     score = minimize_for_world_pose(slam_data.copy(), best_Z, T_world_to_sorigin, Transforms)
#     print(f" Score {score}")
#     return score

# INITIAL_THETA = np.array([0]) # Initial guess on theta is 0 rad
# result = minimize(distance_to_XY_plane_loss, INITIAL_THETA, method = 'Nelder-Mead')
# result_theta = result.x[0] # Result has a bizzare projection instead of rotation effect when included in pose chain?
# # Didn't really have time to debug this.
# print(f" Selected Theta: {result.x[0] * 180/np.pi}")
# print("Final loss:", result.fun)
# print(f" Initial Rotation {R_prior}")
# Transforms.T_world_to_sorigin = rotate_about_world_x(T_prior, result_theta)
# print(f" Final rotation { Transforms.T_world_to_sorigin[:3, :3]}")


# T_world_to_body = T_cam1_to_rx x T_sorigin_to_sbody x T_world_to_sorigin
def get_T_world_to_body(T_sorigin_to_sbody): # A function because I re-use this a lot
    T_world_to_body = (
                    Transforms.T_world_to_sorigin 
                    @ T_sorigin_to_sbody 
                    @ Transforms.T_cam1_to_rx
    )
    return T_world_to_body

### Write IMU data to its own csv file, and to all_data
imu_csv = []
for j in topic_to_processing['/camera/camera/imu'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v)
    imu_csv.append(csv_row)
with open(f'{out_slam}/imu_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(imu_csv))

### Write SLAM camera trajectory
slam_poses_slam_frame = [] # This is a list of T_sorigin_to_sbody
slam_pose_counter = 0
for i in range(slam_data.shape[0]-1):
    T_sorigin_to_sbody = slam_quat_to_HTM(slam_data[i,:])
    slam_poses_slam_frame.append( [slam_data[i,0]] + list(T_sorigin_to_sbody.flatten()) )
    slam_pose_counter += 1


# Using the SLAM trajectory (body pose in the slam frame), and known AoA timestamps
# Compute an interpolated SLAM trajectory of the body pose in the world frame,
# s.t. each pose is timestamp aligned to each AoA measurement.

N_POINTS = 100

body_poses_world_frame = [] #Poses of body in world frame, interpolated to match AoA measurements.
for i in range(t_router.shape[0]-1):

    # Get the closest SLAM measurements to the router timestamp
    tdiffs = np.abs(slam_data[:,0] - t_router[i])
    slam_idx1 = np.argmin(tdiffs)
    tdiffs[slam_idx1] = np.inf
    slam_idx2 = np.argmin(tdiffs)
    istart, iend = sorted([slam_idx1, slam_idx2]) # Make sure indices are ascending

    # Make sure poses we're interpolating between are for the body in the world frame
    current_pose = get_T_world_to_body(slam_quat_to_HTM(slam_data[istart, :]))
    next_pose = get_T_world_to_body(slam_quat_to_HTM(slam_data[iend, :]))

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

    world_frame_pose = np.eye(4)
    world_frame_pose[:3,:3] = interpolated_rotations[idx_match].as_matrix()
    world_frame_pose[:3, 3] = interpolated_positions[idx_match]

    body_poses_world_frame.append(world_frame_pose)


### Now convert all AoA vectors to the world frame

all_aoa_vectors_world_frame = []
all_aoa_vectors_world_frame_rotation = []

for i in range(t_router.shape[0]-1):

    aoa_vectors_rx_frame = aoa_rx_frame[i, :] # Should be an array of 3 paths x 3
    n_paths = aoa_vectors_rx_frame.shape[0]

    aoa_vectors_world_frame = np.empty_like(aoa_vectors_rx_frame)

    # Using the known pose of the body in the world frame, at time t
    # Map each path at time t into the world frame.
    T_rx_to_world = np.linalg.inv(body_poses_world_frame[i])

    # This computation takes the 'tip' of the AoA unit vector in rx frame coordinates
    # And maps it to the 'tip' of the AoA unit vector in world frame coordinates.
    # This means that this will return points w.r.t world origin, and not w.r.t rx frame origin in world frame.
    for path_idx in range(n_paths):
        aoa_vectors_world_frame[path_idx,:] = (T_rx_to_world @ np.append(aoa_vectors_rx_frame[path_idx, :], 1))[:3]
    all_aoa_vectors_world_frame.append(aoa_vectors_world_frame)

    # This computation rotates the AoA unit vector from rx frame coordinates into world frame coordinates.
    # This means that the tip of this unit vector will be w.r.t 0,0,0, but its rotation will be
    # consistent with the world axes.
    aoa_vectors_world_frame_rotation = np.empty_like(aoa_vectors_rx_frame)
    for path_idx in range(n_paths):
        aoa_vectors_world_frame_rotation[path_idx, :] = np.linalg.inv(T_rx_to_world[:3,:3]) @ aoa_vectors_rx_frame[path_idx, :]
    all_aoa_vectors_world_frame_rotation.append(aoa_vectors_world_frame_rotation)
    # These will provide the RPY of the vector along world frame axes, as measured at the receiver's origin.

positions_world = []
for body_pose in body_poses_world_frame:
    positions_world.append(body_pose[:3,3].flatten())

aoa_vectors_world = []
for aoa_vector in all_aoa_vectors_world_frame_rotation:
    aoa_vectors_world.append(aoa_vector)

# TODO: rename to whatever keys Saif uses
np.savez(
        outpath+"/roomba_data_world.npz", 
        timestamps = router_data['timestamps'], 
        csi_data = router_data['csi_matrix'],
        aoa_matrix = router_data['aoa_matrix'],
        strength = router_data['strength'],
        aoa_matrix_world=aoa_vectors_world, 
        positions_world=positions_world
        )

# Plot results

body_orientation_stride = 100
aoa_vector_stride = 100

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

positions_world = np.array(positions_world)
aoa_vectors_world = np.array(aoa_vectors_world)
ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2], label='Trajectory', color='blue')
ax.scatter(*positions_world[0], color='green', label='Start')
ax.scatter(*positions_world[-1], color='red', label='End')

def draw_axes(ax, T, length=0.1):
    """Draw coordinate axes from transformation matrix T."""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)

if body_orientation_stride > 0:
    for i in range(0, len(body_poses_world_frame), body_orientation_stride):
        draw_axes(ax, body_poses_world_frame[i], length=0.4)

if aoa_vector_stride > 0:
    length = 0.2
    n_vectors = 1 # Plot first N paths
    for i in range(0, len(positions_world), aoa_vector_stride):
        for j in range(n_vectors):
            origin = positions_world[i]
            tip = origin + aoa_vectors_world[i][j,:]
            ax.quiver(*origin, *tip, color='purple')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Trajectory and Static Coordinate Frames")
ax.view_init(elev=20, azim=45)
ax.legend()
ax.set_xlim(-1, 3)
ax.set_ylim(0,4)
ax.set_zlim(0,2)
plt.show()
pickle.dump(fig, open(outpath+"/trial_viz.pickle", 'wb'))

### Write Infra1 frames to output directory, and provide references in all_data
for j in topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]:
    cv2.imwrite(out_infra1+"/"+j["name"], j["raw"])
    j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
    all_data.append(j_no_image)

### Write Infra2 frames to output directory, and provide references in all_data
for j in topic_to_processing['/camera/camera/infra2/image_rect_raw'][1]:
    cv2.imwrite(out_infra2+"/"+j["name"], j["raw"])
    j_no_image = { k:v for k,v in j.items() if not (k == "raw") }
    all_data.append(j_no_image)



class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if hasattr(obj, '__dict__'):
            return vars(obj)
        return super().default(obj)



### Copy all world information: transforms, anchors, apriltags, to output
with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


# Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
print("Checking frequency of real data")
print(f" Measured SLAM frequency {len(slam_data) / (END-START)}")


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])
json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

json.dump(args.__dict__, open(outpath+"/meta.json", 'w'), cls=NumpyEncoder, indent=1)