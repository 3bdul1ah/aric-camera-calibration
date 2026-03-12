#!/usr/bin/python3
import math
import random

import rclpy
import DR_init

DR_init.__dsr__id    = ""
DR_init.__dsr__model = "m1013"
_node = rclpy.create_node("doosan_robot_controller")
DR_init.__dsr__node = _node

import numpy as np
from DSR_ROBOT2 import movej, movel, posx, posj, set_robot_mode, get_robot_mode, get_current_tool_flange_posx, get_current_rotm, get_current_posj

# CALIB_START_JOINTS = [-2.69952631, -8.35103798, -120.96444702,
#                        0.31203827, -49.69815826,  87.13671112]
CALIB_START_JOINTS = [-6.1852498054504395, -12.219548225402832, -128.64694213867188,
                       2.930152177810669, -38.689674377441406, 82.12944030761719]


def build_calib_poses(center, n_cycles=1, n_per_cycle=9,
                      xy_mm=60.0, z_mm=30.0, tilt_deg=20.0):
    cx, cy, cz, crx, cry, crz = center
    poses = [list(center)]
    for i in range(n_cycles):
        s = (i + 1) / n_cycles
        for j in range(n_per_cycle):
            phi = j * 2.0 * math.pi / n_per_cycle
            poses.append([
                cx  + xy_mm    * s * math.cos(phi),
                cy  + xy_mm    * s * math.sin(phi),
                cz  + z_mm     * s * (random.random() - 0.5),
                crx + tilt_deg * s * math.cos(phi + math.pi / 2.0),
                cry + tilt_deg * s * math.sin(phi + math.pi / 2.0),
                crz,
            ])
    return poses


class DoosanRobot:
    def __init__(self, **kwargs):
        set_robot_mode(1)

    def move_to_calib_start(self):
        set_robot_mode(1)
        cur = get_current_posj()
        diff = max(abs(float(cur[i]) - CALIB_START_JOINTS[i]) for i in range(6))
        if diff < 0.5:
            return True
        q = posj(*[float(v) for v in CALIB_START_JOINTS])
        return movej(q, vel=100, acc=100) == 0

    def get_current_posx(self):
        return list(get_current_tool_flange_posx())

    def move_posx(self, pose_list):
        p = posx(*[float(v) for v in pose_list])
        return movel(p, vel=100, acc=100, ref=0, mod=0) == 0

    def get_ee_matrix(self):
        """Return 4x4 base_T_flange in metres.
        Translation from get_current_tool_flange_posx (mm to m).
        Rotation from get_current_rotm (3x3).
        """
        pos  = get_current_tool_flange_posx()
        rotm = get_current_rotm()
        tvec = np.array([pos[0] / 1000.0, pos[1] / 1000.0, pos[2] / 1000.0])
        rot  = np.array(rotm)
        return np.vstack([np.c_[rot, tvec.reshape(3, 1)], [0, 0, 0, 1]])

