#!/usr/bin/env python
# coding: utf-8
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import transforms3d as tfs
import math
import file_operate
import rospy


def msg_to_opencv(x, y, z, rx, ry, rz):
    tr = np.array((x, y, z))
    rot = tfs.euler.euler2mat(math.radians(
        rx), math.radians(ry), math.radians(rz))
    return rot, tr


def get_sample(cal, tool):
    hand_base_rot = []
    hand_base_tr = []
    marker_camera_rot = []
    marker_camera_tr = []
    for i in range(0, tool.shape[0], 6):
        (mcr, mct) = msg_to_opencv(
            cal[i], cal[i+1], cal[i+2], cal[i+3], cal[i+4], cal[i+5])
        marker_camera_rot.append(mcr)
        marker_camera_tr.append(mct)
        (hbr, hbt) = msg_to_opencv(
            tool[i], tool[i+1], tool[i+2], tool[i+3], tool[i+4], tool[i+5])
        hand_base_rot.append(hbr)
        hand_base_tr.append(hbt)
    return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)


def compute_calibration(samples):
    (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = samples
    if len(hand_world_rot) != len(marker_camera_rot):
        logerr("Different numbers of hand-world and camera-marker samples!")
        raise AssertionError
    method = cv.CALIB_HAND_EYE_TSAI
    hand_camera_rot, hand_camera_tr = cv.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                          marker_camera_tr, method=method)
    result = tfs.affines.compose(np.squeeze(
        hand_camera_tr), hand_camera_rot, [1, 1, 1])
    return result


def get_test_sample(cal, tool):
    result = []
    for i in range(0, tool.shape[0], 6):
        (mcr, mct) = msg_to_opencv(
            cal[i], cal[i+1], cal[i+2], cal[i+3], cal[i+4], cal[i+5])
        (hbr, hbt) = msg_to_opencv(
            tool[i], tool[i+1], tool[i+2], tool[i+3], tool[i+4], tool[i+5])
        camera = tfs.affines.compose(np.squeeze(mct), mcr, [1, 1, 1])
        arm = tfs.affines.compose(np.squeeze(hbt), hbr, [1, 1, 1])
        result.append((arm, camera))
    return result


if __name__ == '__main__':
    rospy.init_node("base_hand_on_eye_calib", anonymous=False)
    file_path = rospy.get_param("/base_hand_on_eye_calib/base_handeye_data")
    if file_path is not None:
        rospy.loginfo("Get param base_handeye_data file path: "+str(file_path))
        if not str(file_path).endswith(".csv"):
            rospy.logerr("The data file not a csv file :"+str(file_path))
            exit()
        tool, cal = file_operate.read_handeye_data(file_path)
        sample = get_sample(cal=cal, tool=tool)
        rmat = compute_calibration(sample)
        test_sample = get_test_sample(cal, tool)
        rospy.loginfo("The Camera To Hand Matrix\n"+str(rmat))
        rospy.loginfo("The Camera To Hand X,Y,Z:"+str(
            rmat[0:3, 3:4].T)+" \tRX,RY,RZ:"+str(tfs.euler.mat2euler(rmat[0:3, 0:3])))
        for i in range(len(test_sample)):
            temp = np.dot(test_sample[i][0], rmat)
            temp = np.dot(temp, test_sample[i][1])
            rospy.loginfo(str(temp[0:3, 3:4].T))
