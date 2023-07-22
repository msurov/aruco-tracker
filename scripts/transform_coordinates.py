import numpy as np
from scipy.linalg import expm, logm
from quat import quat_rot, quat_mul, quat_conj
import json


def pose_compose(pose_ab, pose_bc):
    p_ab, q_ab = pose_ab
    p_bc, q_bc = pose_bc
    q_ac = quat_mul(q_ab, q_bc)
    p_ac = p_ab + quat_rot(q_ab, p_bc)
    return (p_ac, q_ac)

def pose_inverse(pose):
    p, q = pose
    q_inv = quat_conj(q)
    return (-quat_rot(q_inv, p), q_inv)

def load_parameters(path):
    with open(path) as f:
        par = json.load(f)
        return par

def get_json_pose(obj):
    return (np.array(obj['position'], float), np.array(obj['orientation'], float))

def main():
    par = load_parameters('dataset/parameters.json')
    world_cam_pose = get_json_pose(par['camera'])
    world_marker_pose = get_json_pose(par['marker']['pose'][0])
    cam_marker_pose = pose_compose(pose_inverse(world_cam_pose), world_marker_pose)
    print(cam_marker_pose)

main()