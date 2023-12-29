import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
#opencv-contrib-python  4.6.0.66

dist=np.array(([[0.019367, -0.052131, 0.003435, -0.000312, 0.000000]]))

mtx=np.array([[1136.6399 ,    0.     ,  959.06707],
            [0.     , 1136.88727,  539.12713],
            [0.     ,    0.     ,    1.     ]])

cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
marketLength=0.030
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
parameters =  aruco.DetectorParameters_create()

global  rvec, tvec

while True:
    ret, frame = cap.read()

    h1, w1 = frame.shape[:2]  
    # print(h1, w1) #1920 1080
    
    # 读取摄像头画面
    # 纠正畸变
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w1, h1), 1, (w1, h1))
    dst1 = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
        
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame=dst1

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

#    如果找不打id
    if ids is not None:

        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marketLength, mtx, dist)
        # 估计每个标记的姿态并返回值rvet和tvec ---不同
        # from camera coeficcients
        (rvec-tvec).any() # get rid of that nasty numpy value array error

        for i in range(rvec.shape[0]):
            cv2.drawFrameAxes(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
            rvec[i, :, :]=rvec[i, :, :]  * (180.0 / math.pi) #弧度制转为角度制
            # print(tvec,rvec)
            aruco.drawDetectedMarkers(frame, corners)
        ###### DRAW ID #####
        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # 显示结果框架
    resized_image = cv2.resize(frame, (int(1920/2), int(1080/2)))
    cv2.imshow("frame",resized_image )

    key = cv2.waitKey(1)
    # break

    if key == 27:         # 按esc键退出
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break

    if key == ord(' '):   # 按空格键保存
        tvec_str = ', '.join(map(str, tvec[0, :, :].ravel()))
        rvec_str = ', '.join(map(str, rvec[0, :, :].ravel()))
        print(tvec_str,',', rvec_str)
        # filename = str(time.time())[:10] + ".jpg"
        # cv2.imwrite(filename, frame)
'''


while True:
    ret, frame = cap.read()
    
    if ret:
        h, w = frame.shape[:2]
        
        # 畸变校正
        new_camera_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted_frame = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
        
        # ArUco 标记检测
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(undistorted_frame, aruco_dict, parameters=parameters)
        
        if ids is not None and len(ids) > 0:
            # 绘制检测到的标记边界框
            cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
            
            # 获取标记的旋转矩阵和平移矩阵
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 10, new_camera_mtx, dist)
            
            # 在图像中绘制坐标轴
            for i in range(len(ids)):
                cv2.drawFrameAxes(undistorted_frame,new_camera_mtx, dist, rvecs[i, :, :], tvecs[i, :, :], 0.03)
                aruco.drawDetectedMarkers(undistorted_frame, corners)
                #绘制坐标轴
                corner_int = corners[i][0][0].astype(int)
                axis_length = 30  # 定义轴线长度
                axis_end_x = tuple((corner_int + (axis_length * rvecs[i][0][0])).astype(int))
                axis_end_y = tuple((corner_int + (axis_length * rvecs[i][0][1])).astype(int))
                axis_end_z = tuple((corner_int + (axis_length * rvecs[i][0][2])).astype(int))
                # 绘制自定义的 XYZ 轴线条
                # 绘制 X 轴（蓝色）
                cv2.line(undistorted_frame, corner_int, axis_end_x, (255, 0, 0), 3)
                # 绘制 Y 轴（绿色）
                cv2.line(undistorted_frame, corner_int, axis_end_y, (0, 255, 0), 3)
                # 绘制 Z 轴（红色）
                cv2.line(undistorted_frame, corner_int, axis_end_z, (0, 0, 255), 3)
        
        
        # 显示原始图像和校正后的图像
        cv2.imshow('Original', frame)
        cv2.imshow('Undistorted', undistorted_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
'''