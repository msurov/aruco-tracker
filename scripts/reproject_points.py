import cv2
import matplotlib.pyplot as plt
from os import listdir
from os.path import join, split
import json
import numpy as np
from dataclasses import dataclass
from quat import quat_mul, quat_rot, quat_conj
import argparse


@dataclass
class Pose:
    p : np.ndarray
    q : np.ndarray

    def __post_init__(self):
        self.p = np.array(self.p, float).reshape(3)
        self.q = np.array(self.q, float).reshape(4)

def compose(pose_ab : Pose, pose_bc : Pose):
    return Pose(
        q = quat_mul(pose_ab.q, pose_bc.q),
        p = quat_rot(pose_ab.q, pose_bc.p) + pose_ab.p
    )

def inverse(pose : Pose):
    q_inv = quat_conj(pose.q)
    return Pose(
        p = quat_rot(q_inv, -pose.p),
        q = q_inv
    )

def transform(pose : Pose, vec : np.ndarray):
    return quat_rot(pose.q, vec) + pose.p

@dataclass
class Marker:
    pose : Pose
    side : float

@dataclass
class CameraParameters:
    K : np.ndarray
    pose : Pose

def load_parameters(path):
    with open(f'{path}/parameters.json') as f:
        return json.load(f)

def get_dataset(path):
    files = listdir(path)
    endings = '.png', '.bmp', '.jpg'
    files = filter(lambda f: f.endswith(endings), files)
    files = sorted(files)
    return [join(path, f) for f in files]

def load_pose(obj):
    p = obj['position']
    q = obj['orientation']
    return Pose(p, q)

def project_points(K, points):
    shape = points[...,0].shape
    q = K @ points.T
    u = np.zeros(shape + (2,), float)
    u[...,0] = q[0,...] / q[2,...]
    u[...,1] = q[1,...] / q[2,...]
    return u

def plot_contour(c):
    c = np.concatenate((c, [c[0,...]]), axis=0)
    plt.plot(c[:,0], c[:,1])

def process_sample(impath : str, marker : Marker, camera : CameraParameters):
    plt.figure(impath)
    im = cv2.imread(impath, 0)
    plt.imshow(im, cmap='gray', interpolation='nearest')
    cam_marker_pose = compose(inverse(camera.pose), marker.pose)
    points = np.array([
        [0, 0, 0],
        [marker.side, 0, 0],
        [marker.side, marker.side, 0],
        [0, marker.side, 0],
    ], float)
    points = transform(cam_marker_pose, points)
    u = project_points(camera.K, points)
    plot_contour(u)

def main():
    ap = argparse.ArgumentParser(description='Generate sample routes and trajectories')
    ap.add_argument('-i', '--inp', type=str, required=True, help='path to the folder with source files xxx.png and parameters.json')
    ap.add_argument('-f', '--file', type=str, required=False, default=None, help='if specified the script processes this file only')
    args = ap.parse_args()

    input_dir = args.inp

    files = get_dataset(input_dir)
    par = load_parameters(input_dir)
    campar = CameraParameters(
        K = np.array(par['camera']['intrinsics'], float),
        pose = load_pose(par['camera'])
    )
    marker_side = float(par['marker']['size'])
    markers = [
        Marker(pose = load_pose(obj), side = marker_side) 
        for obj in par['marker']['pose']
    ]

    for impath, marker in zip(files, markers):
        if args.file is not None:
            if split(impath)[1].find(args.file) < 0:
                continue
        process_sample(impath, marker, campar)

    plt.show()

main()
