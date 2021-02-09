#!/usr/bin/env python
# coding:utf-8
import rospy
import transforms3d as tfs
from geometry_msgs.msg import PoseStamped
from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
import math
import time
import file_operate
from tabulate import tabulate


real_jaka_pose = None
real_camera_pose = None


def jaka_callback(pose):
    global real_jaka_pose
    real_jaka_pose = pose.pose


def camera_callback(pose):
    global real_camera_pose
    real_camera_pose = pose.pose


def get_pose_from_ros(pose):
    eulor = tfs.euler.quat2euler(
        (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
    real_pose = [pose.position.x, pose.position.y, pose.position.z,
                 math.degrees(eulor[0]), math.degrees(eulor[1]), math.degrees(eulor[2])]
    return real_pose


def get_csv_from_sample(samples):
    data = ""
    for d in samples:
        data += str("hand,"+str(get_pose_from_ros(d['robot']))[1:-1]+"\n")
        data += str("eye,"+str(get_pose_from_ros(d['optical']))[1:-1]+"\n")
    return data


def cauculate(samples,hand_calib):
    esti_pose = {}
    save_data = ""
    if len(samples) > 2:
        data =  [['algoritihms','x','y','z','rx','ry','rz',"distance"]]
        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            pose,final_pose = hand_calib.compute_calibration(samples,algorithm=algoram)
            data.append([algoram,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],hand_calib._distance(pose[0],pose[1],pose[2])])
            esti_pose[algoram] = final_pose

        print  "\n"+tabulate(data,headers="firstrow") + "\n"
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        test_result =  hand_calib._test_data(data[1:])
        data = [['name','x','y','z','rx','ry','rz',"distance"]]
        for d in test_result:
            data.append(d)
        print tabulate(data,headers="firstrow")
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            print tabulate(esti_pose[algoram],headers="firstrow")
            save_data  += str(  "\n"+tabulate(esti_pose[algoram],headers="firstrow") + "\n")
    return save_data


def save(result_path,save_data):
        if result_path is not None:
            file_operate.save_file(result_path,save_data)
            rospy.loginfo("Save result to  "+str(result_path))
        print get_csv_from_sample(samples)



def check_data():
    while not rospy.is_shutdown():
        time.sleep(1)
        if real_jaka_pose == None:
            rospy.loginfo('Waiting jaka pose topic data ' + jaka_pose_topic)
        elif real_camera_pose == None:
            rospy.loginfo('Waiting camera pose topic data ' +
                          camera_pose_topic)
        else:
            break



def init():
    jaka_pose_topic = rospy.get_param(
        "/jaka_hand_on_eye_calib/jaka_pose_topic")
    camera_pose_topic = rospy.get_param(
        "/jaka_hand_on_eye_calib/camera_pose_topic")
    rospy.loginfo("Get topic from param server: jaka_pose_topic:" +
                  str(jaka_pose_topic)+" camera_pose_topic:"+str(camera_pose_topic))
    rospy.Subscriber(jaka_pose_topic, PoseStamped, jaka_callback)
    rospy.Subscriber(camera_pose_topic, PoseStamped, camera_callback)



if __name__ == '__main__':
    rospy.init_node("jaka_hand_on_eye_calib", anonymous=False)
    hand_calib = HandeyeCalibrationBackendOpenCV()

    check_data()
    samples = []

    while not rospy.is_shutdown():
        command = str(raw_input("input:  r     record,c    calculate,s     save,q    quit:"))

        if command == "r":
            samples.append( {"robot": real_jaka_pose, "optical": real_camera_pose})
            print "current sample size:"+str(len(samples))
            if len(samples) > 2:
                temp_sample = hand_calib.compute_calibration(samples)
                print temp_sample

        elif command == 'c':
            cauculate(samples,hand_calib)

        elif command == 'q':
            break

        elif command == 's':
            save()
        
