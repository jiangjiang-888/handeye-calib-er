#!/usr/bin/env python
# coding: utf-8
import rospy
from geometry_msgs.msg import Pose


def jaka_callback(pose):
    print pose


def camera_callback(pose):
    # print pose
    pass


if __name__ == '__main__':
    rospy.init_node("jaka_hand_on_eye_calib", anonymous=False)
    jaka_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/jaka_pose_topic")
    camera_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/camera_pose_topic")

    rospy.loginfo("Get topic from param server: jaka_pose_topic:"+str(jaka_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))

    rospy.Subscriber(jaka_pose_topic, Pose, jaka_callback)
    rospy.Subscriber(camera_pose_topic, Pose, camera_callback)
    while not rospy.is_shutdown():
        command = str(raw_input("input r to record,c to calculate,q to quit:"))
        if command == "r" :
            print "record"
        elif command=='c' :
            print "calculate"