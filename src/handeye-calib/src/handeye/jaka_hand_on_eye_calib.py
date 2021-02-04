#!/usr/bin/env python
# coding: utf-8
import rospy
import transforms3d as tfs
from geometry_msgs.msg import Pose
import math
from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
real_jaka_pose = None
real_camera_pose = None



def jaka_callback(pose):
    global real_jaka_pose
    eulor = tfs.euler.quat2euler((pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z))
    real_jaka_pose = [pose.position.x,pose.position.y,pose.position.z,eulor[0]/math.pi*180,eulor[1]/math.pi*180,eulor[2]/math.pi*180]


def camera_callback(pose):
    global real_camera_pose
    eulor = tfs.euler.quat2euler((pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z))
    real_camera_pose = [pose.position.x,pose.position.y,pose.position.z,eulor[0]/math.pi*180,eulor[1]/math.pi*180,eulor[2]/math.pi*180]


if __name__ == '__main__':
    rospy.init_node("jaka_hand_on_eye_calib", anonymous=False)
    
    HandEyeCal = HandeyeCalibrationBackendOpenCV()

    jaka_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/jaka_pose_topic")
    camera_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/camera_pose_topic")
    rospy.loginfo("Get topic from param server: jaka_pose_topic:"+str(jaka_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))

    rospy.Subscriber(jaka_pose_topic, Pose, jaka_callback)
    rospy.Subscriber(camera_pose_topic, Pose, camera_callback)

    samples = []

    while not rospy.is_shutdown():
        command = str(raw_input("input r to record,c to calculate,q to quit:"))
        if command == "r" :
            print "record"
            samples.append({"robot":real_jaka_pose,"optical":real_camera_pose})
        elif command=='c' :
            print "calculate"
        elif command=='l' :
            print samples
        elif command=='q' :
            exit()
        elif command=='s' :
            data = ""
            for d in samples:
                data += str("hand,"+str(d['robot'])[1:-1]+"\n")
                data += str("eye,"+str(d['optical'])[1:-1]+"\n")
            print data
            # fo = open("data.csv", "w")
            # fo.write(data)
            # fo.close()