#iai_kinect2's convert_calib_pose_to_urdf_format.py
#from: https://github.com/code-iai/iai_kinect2/blob/master/kinect2_calibration/scripts/convert_calib_pose_to_urdf_format.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import os
import tempfile
import tf
import yaml

def read_calib_pose(fname):
    tmp = tempfile.TemporaryFile()
    # we need modify original yaml file because yaml.load(fname) simply will fail
    with open(fname, "r") as f:
        reader = f.readlines()
        for row in reader:
            if row[0] == "%":
                # remove first line: "%YAML:1.0"
                continue
            a = row.find("!!")
            if row.find("!!") != -1:
                # remove "!!opencv-matrix"
                row = row[:row.find("!!")] + os.linesep
            tmp.write(row)
    tmp.seek(0)
    data = yaml.load(tmp)
    return data

def calc_xyz_rpy(data):
    mat = np.resize(data["rotation"]["data"], (3, 3))
    xyz = data["translation"]["data"]
    rpy = tf.transformations.euler_from_matrix(mat)
    return xyz, rpy

def print_urdf(xyz, rpy):
    print("""
    <joint name=\"kinect2_rgb_joint\" type=\"fixed\">
      <origin xyz=\"{x} {y} {z}\" rpy=\"{roll} {pitch} {yaw}\"/>
      <parent link=\"kinect2_rgb_optical_frame\"/>
      <child link=\"kinect2_ir_optical_frame\"/>
    </joint>
    """.format(x=xyz[0], y=xyz[1], z=xyz[2],
             roll=rpy[0], pitch=rpy[1], yaw=rpy[2]))
