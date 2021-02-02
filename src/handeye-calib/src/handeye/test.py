#!/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np
import transforms3d as tfs
import math

def msg_to_opencv(x,y,z,rx,ry,rz):
    tr = np.array((x,y,z))
    rot = tfs.euler.euler2mat(math.radians(rx),math.radians(ry),math.radians(rz))
    return rot,tr

cal = np.array((-0.105,0.002,0.489,-179.62,0.35,-88.96,0.099,0.078,0.466,-177.53,5.20,-46.09,-0.057,0.068,0.510,175.61,-7.30,-81.18))
tool = np.array((-336.36003292704021/1000,8.7522381567008427/1000,542.11399000000006/1000,-179.99000000089154,0.0028000003654367629,136.66599999999994,-304.07819121653648/1000,-182.95819039516834/1000,517.74147217452071/1000,175.88096766521406,4.9199860791433414,179.53624633225331,-324.38358819547636/1000,-4.0395468086953583/1000,563.1148423912507/1000,-172.54759950350569,-3.5936581663462164,144.24443969104817))

# 获取Sample
def get_sample(cal,tool):
    hand_base_rot = []
    hand_base_tr = []
    marker_camera_rot = []
    marker_camera_tr = []
    for i in range(0,tool.shape[0],6):
        (mcr, mct) = msg_to_opencv(cal[i],cal[i+1],cal[i+2],cal[i+3],cal[i+4],cal[i+5])
        marker_camera_rot.append(mcr)
        marker_camera_tr.append(mct)

        (hbr, hbt) = msg_to_opencv(tool[i],tool[i+1],tool[i+2],tool[i+3],tool[i+4],tool[i+5])
        hand_base_rot.append(hbr)
        hand_base_tr.append(hbt)

    return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

#计算标定结果
def compute_calibration(samples):
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = samples
        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        method = cv.CALIB_HAND_EYE_TSAI
        hand_camera_rot, hand_camera_tr = cv.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                               marker_camera_tr, method=method)
        result = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])
        return result

sample = get_sample(cal=cal,tool=tool)


rmat = compute_calibration(sample)


def get_test_sample(cal,tool):
    result = []
    for i in range(0,tool.shape[0],6):
        (mcr, mct) = msg_to_opencv(cal[i],cal[i+1],cal[i+2],cal[i+3],cal[i+4],cal[i+5])
        (hbr, hbt) = msg_to_opencv(tool[i],tool[i+1],tool[i+2],tool[i+3],tool[i+4],tool[i+5])
        camera = tfs.affines.compose(np.squeeze(mct), mcr, [1, 1, 1])
        arm = tfs.affines.compose(np.squeeze(hbt), hbr, [1, 1, 1])
        result.append((arm,camera))
    return result

test_sample = get_test_sample(cal,tool)

test_sample[0][0]*rmat*test_sample[0][1]
temp = np.dot(test_sample[0][0],rmat)
temp = np.dot(temp,test_sample[0][1])
print(temp)


temp[0:3,3:4].T*1000






