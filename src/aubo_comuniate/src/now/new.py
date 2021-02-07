from robotcontrol import *
#from capture3d import *
import time
import libpyauboi5
import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Process, Queue
import os
import math 

import capture3d
import numpy as np
from numpy import dot
from numpy import mat
from numpy.linalg import inv
from pyquaternion import Quaternion
import bbb

def run(robot,pose):
    tool =  { "pos": (-0.073136, -0.007195, 0.092327), "ori": (1.0, 0.0, 0.0, 0.0) }
    
    r = robot.get_current_waypoint()
    r = robot.base_to_base_additional_tool( r['pos'], r['ori'],tool)
    r1 = bbb.getTarget(
        [
            r['pos'],
            r['ori']
        ],
        [
            pose['pos'],
            pose['ori']
            # [-0.0829281285405,0.0183543693274, 0.552563965321],
            # [0.562188117937,-0.526761407173,0.429122187468,-0.471509372321]
        ]
    )
    pos,rpy = bbb.get_cartesian_Target( robot, r1)

    # print(pos,rpy)

    robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_01, 1)
    user_io_status = robot.get_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_01)
                
    robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_00, 0)

    robot.move_to_target_in_cartesian(pos,rpy)
    time.sleep(0.5)

    pos[2] += 0.9

    robot.move_to_target_in_cartesian(pos,rpy)

    pos[2] -= 0.9
    robot.move_to_target_in_cartesian(pos,rpy)

    robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_01, 0)
    user_io_status = robot.get_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_01)
                
    robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_00, 1)

    pos[2] += 0.9
    robot.move_to_target_in_cartesian(pos,rpy)

    

def test_robot():
    # 初始化logger
    logger_init()
    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))
    # 系统初始化
    Auboi5Robot.initialize()
    # 创建机械臂控制类
    robot = Auboi5Robot()
    # 创建上下文
    handle = robot.create_context()
    try:
        # 链接服务器
        ip = '10.55.17.38'
        port = 8899
        result = robot.connect(ip, port)
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 设置碰撞等级
            #robot.set_collision_class(7)
            # 设置关节最大加速度
            robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
            # 设置关节最大加速度
            robot.set_joint_maxvelc((0.1, 0.1, 0.1, 0.1, 0.1, 0.1))
            #取箱位姿信息
            # move_fetch = capture3d.connectScanner('box')
            move_fetch = {'msg_type': 'location_rsp', 'obj_type': 'box', 'num': 1, 'pose_list': [{'x': -108.289635, 'y': -63.525009, 'z': 866.706482, 'rx': 0.0, 'ry': 0.0, 'rz': 87.137596}]}
            print("pose_num",move_fetch['num'])
            if move_fetch['num']==0 :
                print("pose_num error !")
            else:
                xyz=[move_fetch['pose_list'][0]['x']/1000,move_fetch['pose_list'][0]['y']/1000,move_fetch['pose_list'][0]['z']/1000]
                print("xyz",xyz)
                rx,ry,rz= math.radians(move_fetch['pose_list'][0]['rx']), math.radians(move_fetch['pose_list'][0]['ry']), math.radians(move_fetch['pose_list'][0]['rz'])
                quater=robot.rpy_to_quaternion([rx,ry,rz])
                pose={"pos":xyz,"ori":quater}
                print("pose",pose)
                run(robot,pose)
            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
    finally:
        if robot.connected:
            logger.info("{0} test shutdown and disconnect.".format(Auboi5Robot.get_local_time()))
            robot.disconnect()
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

test_robot()
