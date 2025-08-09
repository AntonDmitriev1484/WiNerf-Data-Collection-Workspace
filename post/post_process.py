
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

# Example usage:
# python3 post_process.py -t stereoi_sq -c cam_target_daslab -a pilot3/anchors.json -p pilot3/apriltags.json -i 10


parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)
parser.add_argument("--cam_calibration_file", "-c", type=str)
parser.add_argument("--crop_start", type=float) # Pass the ROS timestamp that you want to crop away all data before. Data will still be used to compute transforms.
parser.add_argument("--apriltags_file", "-p", type=str)
parser.add_argument("--override_april_start", type=str )
parser.add_argument("--in_aoa", "a", type=str) # Absolute file path of timestamped AoA CSV file.

args = parser.parse_args()

outpath = f'./out/{args.trial_name}_post'

out_infra1 = f'{outpath}/infra1'
out_infra2 = f'{outpath}/infra2'
out_slam = f'{outpath}/slam'

os.makedirs(outpath, exist_ok=True)
os.makedirs(out_infra1, exist_ok=True)
os.makedirs(out_infra2, exist_ok=True)
os.makedirs(out_slam, exist_ok=True)

in_aoa = args.in_aoa
in_slam = f'../orbslam/out/{args.trial_name}_cam_traj.txt'
in_slam_kf = f'../orbslam/out/{args.trial_name}_kf_traj.txt'
in_kalibr = f"../kalibr/camimu_out/{args.cam_calibration_file}-camchain-imucam.yaml"
in_apriltags = f"../world/{args.apriltags_file}"
in_anchors = f"../world/{args.anchors_file}"

bagpath = Path(f'../collect/ros2/{args.trial_name}')

slam_kf_data = np.loadtxt(in_slam_kf)
slam_kf_data[:,0] *= 1e-9
slam_data = np.loadtxt(in_slam)
slam_data[:,0] *= 1e-9 # Adjust timestamps to be in 's'

aoa_data = np.loadtxt(in_aoa)

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
print(f"ROS duration {START} - {END}")
print(f"Data start {START} cropped to {args.crop_start}")

def filtt(arr): # For filtering a json output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x["t"]), arr)) # First filter by crop
    return list(filter(lambda x: (START <= x["t"] <= END), arr)) # Then filter by ros timestamps
def filtt2(arr): # For filtering a CSV output
    if args.crop_start is not None: arr = list(filter(lambda x: (args.crop_start <= x[0]), arr))
    return list(filter(lambda x: (START <= x[0] <= END), arr))


### Define all coordinate transforms
Transforms = SimpleNamespace()
Transforms.T_body_to_imu = np.array([
                                        [1, 0, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 0, 1]
                                    ])

Transforms.T_body_to_decawave = np.eye(4)
Transforms.T_body_to_decawave[:3,3] = np.array([-0.12, 0.015, -0.1])


Transforms.T_cam1_to_rx = np.array([]) # TODO
Transforms.T_body_to_cam1 = np.linalg.inv(Transforms.T_cam1_to_rx)

infra1_raw_frames = topic_to_processing['/camera/camera/infra1/image_rect_raw'][1]
Transforms = extract_apriltag_pose(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)
# Transforms = extract_apriltag_pose_PnP(slam_data, infra1_raw_frames, Transforms, in_kalibr, in_apriltags)

if args.override_april_start is not None:
    Transforms.T_slam_world[:3, 3] = np.array(json.loads(args.override_april_start))


# T_world_to_body = T_body_to_imu^-1 x T_imu_to_sbody^-1 x T_sorigin_to_sbody x T_world_to_sorigin
def get_T_world_to_body(T_sorigin_to_sbody): # A function because I re-use this a lot
    T_world_to_body = (
                    Transforms.T_world_to_sorigin 
                    @ T_sorigin_to_sbody 
                    @ np.linalg.inv(Transforms.T_imu_to_sbody) 
                    @ np.linalg.inv(Transforms.T_body_to_imu)
    )
    return T_world_to_body

### Write IMU data to its own csv file, and to all_data
imu_csv = []
for j in topic_to_processing['/camera/camera/imu'][1]:
    csv_row = []
    for k, v in j.items(): csv_row.append(v)
    imu_csv.append(csv_row)
    all_data.append(j)
with open(f'{out_slam}/imu_data.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(imu_csv))

### Write SLAM camera trajectory
body_poses_world_frame = [] # More accurately, this is a list of T_world_to_body
slam_poses_slam_frame = [] # This is a list of T_sorigin_to_sbody
slam_pose_counter = 0

all_data_synthetic = [] # Keep interpolated points in a separate file from all.json

for i in range(slam_data.shape[0]-1):

    T_sorigin_to_sbody = slam_quat_to_HTM(slam_data[i,:])
    slam_poses_slam_frame.append( [slam_data[i,0]] + list(T_sorigin_to_sbody.flatten()) )

    T_world_to_body = get_T_world_to_body(T_sorigin_to_sbody)

    body_poses_world_frame.append( [slam_data[i,0]] + list(T_world_to_body.flatten()) )

    # NOTE: Not changing these field names because I don't want to blow up all downstream programs
    j = {
        "t": slam_data[i,0],
        "type": "slam_pose",
        "T_body_slam" : T_sorigin_to_sbody,
        "T_body_world" : T_world_to_body
    }
    all_data.append(j) # Append GT data into the sensor stream to use as Pose3 corrections
    slam_pose_counter += 1


with open(f'{out_slam}/body_poses_world_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(body_poses_world_frame))
with open(f'{out_slam}/slam_poses_slam_frame.csv', 'w') as fs: csv.writer(fs).writerows(filtt2(slam_poses_slam_frame))


### Now convert all AoA data to the world frame

body_poses_world_frame_AoA_aligned = [] # Let this be a list of HTMs, rather than timestamp + flattened HTM
# We'll interpolate a slam pose to match an AoA measurement timestamp

aoa_vectors_world_frame = []
# Assuming each slam pose is perfectly aligned with AoA!!
for i in range(aoa_data.shape[0]-1):

    aoa_rx_frame = aoa_data[i, :]
    aoa_world_frame = body_poses_world_frame_AoA_aligned[i] @ aoa_rx_frame

    aoa_vectors_world_frame.append(aoa_world_frame)

# TODO, log body_poses_world_frame_AoA_aligned in TUM format in CSV
# log Aoa_vectors_world frame in rpy in CSV?






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
shutil.copy(in_anchors, f'{outpath}/anchors.json')
shutil.copy(in_apriltags, f'{outpath}/apriltags.json')
with open(f'{outpath}/transforms.json', 'w') as fs: json.dump(vars(Transforms), fs, cls=NumpyEncoder, indent=1)


# Run sanity check to make sure measurements are at the frequency we expect them to be before testing in the graph
print("Checking frequency of real data")
print(f" Measured SLAM frequency {len(slam_data) / (END-START)}")


# Filter to make sure all messages ( and data jsons ) fall within the ROS recording time interval, (because some of them don't apparently)
all_data = filtt(all_data)
all_data = sorted(all_data, key=lambda x: x["t"])
json.dump(all_data, open(outpath+"/all.json", 'w'), cls=NumpyEncoder, indent=1)

json.dump(args.__dict__, open(outpath+"/meta.json", 'w'), cls=NumpyEncoder, indent=1)