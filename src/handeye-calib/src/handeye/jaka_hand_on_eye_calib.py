#!/usr/bin/env python
# coding: utf-8
import rospy


if __name__ == '__main__':
    rospy.init_node("jaka_hand_on_eye_calib", anonymous=False)
    jaka_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/jaka_pose_topic")
    camera_pose_topic = rospy.get_param("/jaka_hand_on_eye_calib/camera_pose_topic")
    rospy.loginfo("Get topic from param server: jaka_pose_topic:"+str(jaka_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))