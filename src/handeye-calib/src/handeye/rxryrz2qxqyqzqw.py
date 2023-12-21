import transforms3d as tfs
import numpy as np

# 假设你有一个位移向量和欧拉角
translation = [-0.0306808 , -0.0679145 ,  0.591702]  # x, y, z
euler_angles = [ -2.9596  ,  0.143867  ,   0.165729 ]  # rx, ry, rz

# 将欧拉角转换为旋转矩阵
rotation_matrix = tfs.euler.euler2mat(euler_angles[0], euler_angles[1], euler_angles[2], 'sxyz')

# 创建一个四元数
quaternion = tfs.quaternions.mat2quat(rotation_matrix)
print(rotation_matrix)
print("Quaternion:", quaternion)
# 结果和鱼香ros顺序一样
# [ 0.63593528 -0.76817245  0.0733952  -0.01051461]