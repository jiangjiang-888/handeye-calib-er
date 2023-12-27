#创建二维码

import cv2
import numpy as np
from cv2 import aruco
#opencv-contrib-python  4.6.0.66
#旧版本和新版本差异较大,较高的版本aruco的函数名称会有比较大的变化，所以这里最高用4.6.0,生成的图标和https://chev.me/arucogen/上的一样

# 生成aruco标记
# 加载预定义的字典
dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
# 生成标记
markerImage = np.zeros((200, 200), dtype=np.uint8)
# aruco.drawCharucoDiamond(dictionary=dictionary,ids=995,squareLength=200,markerLength=100)
markerImage = aruco.drawMarker(dictionary, 995, 200, markerImage, 1)
cv2.imwrite("marker995.png", markerImage)
