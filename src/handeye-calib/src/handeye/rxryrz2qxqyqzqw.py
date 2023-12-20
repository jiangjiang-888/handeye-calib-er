import transforms3d as tfs
import numpy as np

# 假设你有一个位移向量和欧拉角
translation = [-0.108712  ,  -0.0384386  , 0.549091]  # x, y, z
euler_angles = [ -1.76351  , 0.077272 ,   -0.126851 ]  # rx, ry, rz

# 将欧拉角转换为旋转矩阵
rotation_matrix = tfs.euler.euler2mat(euler_angles[0], euler_angles[1], euler_angles[2], 'sxyz')

# 创建一个四元数
quaternion = tfs.quaternions.mat2quat(rotation_matrix)

print("Quaternion:", quaternion)
# 结果和鱼香ros顺序一样
# [ 0.63593528 -0.76817245  0.0733952  -0.01051461]