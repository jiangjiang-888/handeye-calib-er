# 基于ROS的手眼标定程序包
## 概览
- 如果教程对你有帮助，可以start一下~
- 包含基础标定程序包，提供多组机器臂工具坐标和Marker坐标即可完成标定
- 包含JAKA、AUBO机械臂标定程序
- 本程序在`ros kinetic melodic`平台测试通过

>本程序包目前仅针对眼在手上的标定，通过输入两组以上的机械臂姿态信息(x,y,z,rx,ry,rz)和装在机械手上的相机所识别的标志物的姿态信息，经过程序计算可输出，机械臂末端和相机之间的坐标变换矩阵。

```mermaid
graph LR
    A[机械臂位姿] -->C[手眼标定]
    B[相机中标定板位姿] -->C
    C --> F[末端与相机位姿]
```

## 使用指南
### 1、基础使用
基础使用是在得到多组机械臂位姿与机械臂末端相机位姿之后直接使用本程序进行计算机械臂末端与相机之间的位姿关系。

- 机械臂位姿可以通过示教器或者SDK进行获取
  
- 相机中标定板位姿我们可以通过ArUco或者ArTookit等工具获得，可以参考这里。

> 我们使用一般读到的`（X,Y,Z,RX,RY,RZ）`六个数据表示

#### 安装测试



1. 下载编译
    ```
    git clone http://10.55.16.230/sangxin/handeye-calib.git
    cd handeye-calib
    catkin_make or catkin build
    ```
2. 修改base_hand_on_eye_calib.launch文件中，base_handeye_data参数为从机械臂位姿和标定板位姿所在的配置文件所在的绝对目录,可以使用launch文件的`find`。
    ```
    <launch>
      <!-- <arg   name="base_handeye_data"   default="The file path of handeye data." /> -->
      <arg   name="base_handeye_data"   default="$(find handeye-calib)/config/base_hand_on_eye_test_data.csv" />
      <node pkg="handeye-calib" type="base_hand_on_eye_calib.py" name="base_hand_on_eye_calib" output="screen" >
          <param name="base_handeye_data" value="$(arg base_handeye_data)" />
      </node>
    </launch>
    ```
3. 运行程序
    ```
    source devel/setup.bash
    roslaunch handeye-calib base_hand_on_eye_calib.launch
    ```

4. 查看结果
    程序会根据配置文件中的坐标进行计算，最终输出如下数据(单位毫米，弧度制)：
    ```
    [INFO] [1612332792.280545]: The Camera To Hand Matrix
    [[ 0.67778048 -0.73397949  0.04344798  0.10180987]
    [ 0.73514963  0.67753423 -0.02241383  0.05040806]
    [-0.0129862   0.04713243  0.99880423 -0.0038281 ]
    [ 0.          0.          0.          1.        ]]

    [INFO] [1612332792.284192]: The Camera To Hand 
    X,Y,Z:[[ 0.10180987  0.05040806 -0.0038281 ]]    
    RX,RY,RZ:(0.04715387459563979, 0.012986569559696204, 0.8259788402742381)

    [INFO] [1612332792.288997]: The CheckBoard1 in the base is:
    [[-0.99814102 -0.02938217  0.05339659 -0.39806312]
    [ 0.0290525  -0.9995538  -0.0069399   0.01683085]
    [ 0.05357667 -0.0053757   0.99854927  0.05607137]
    [ 0.          0.          0.          1.        ]]
    XYZ:[[-0.39806312  0.01683085  0.05607137]]
    ```

### 2、结合JAKA机械臂使用
- **jaka标定文件会自己订阅两个话题的数据**，一个是机械臂的位姿话题和相机中标定物的位姿话题。
- 机械臂的话题可以通过运行本仓库中的jaka_comuniate功能包中的jaka_comuniate.launch获得。
- 相机中标记物的姿态数据，可以参考本文第四节使用ArUco获取标定板位姿

#### 1.配置jaka机械臂ip地址信息

配置jaka_host参数为你的jaka机械臂所在的host，并确保你目前所使用的电脑能够`ping`通该ip。

运行该节点后将会发布`jaka_pose`话题并运行

```
<launch>
    <arg  name="jaka_host"   default="10.55.17.17" />
    <node pkg="jaka_comuniate" type="jaka_comuniate" name="jaka_comuniate" output="screen" >
         <param name="jaka_host" value="$(arg jaka_host)" />
    </node>
</launch>
```

运行节点

```
source devel/setup.bash
roslaunch jaka_comuniate jaka_comuniate.launch 
```
#### 2.运行标定板识别程序
  请参考第四节（4.使用系统ArUco获取标定板位姿）

#### 3.配置话题信息
主要配置参数有`jaka_pose_topic`、`camera_pose_topic`。分别代表jaka机械臂的通信地址和，相机中标记物的位姿话题。
  ```
  <launch>
    <!-- The arm tool Pose Topic,Use ros geometry_msgs::Pose-->
    <arg   name="jaka_pose_topic"   default="/jaka_pose" />
    <!-- The arm marker in camera Pose Topic,Use ros geometry_msgs::Pose-->
    <arg   name="camera_pose_topic"   default="/ar_pose_estimate/marker_to_camera" />

    <node pkg="handeye-calib" type="jaka_hand_on_eye_calib.py" name="jaka_hand_on_eye_calib" output="screen" >
         <param name="jaka_pose_topic" value="$(arg jaka_pose_topic)" />
         <param name="camera_pose_topic" value="$(arg camera_pose_topic)" />
    </node>
    
</launch>
  ```

#### 3.运行标定程序

```
source devel/setup.bash
roslaunch handeye-calib jaka_hand_on_eye_calib.launch
```

#### 4.开始标定
程序运行是会对话题数据进行检测，先检测是否收到机械臂数据，如果没有会一直等待。
当检测到已经接收到数据之后，就会出现，命令提示。
命令定义如下：
```
r  rocord    记录一组手眼数据（当记录的数据大于程序计算所需的数据量之后会进行自动计算）
c  calculate 计算当前数据
s  save      保存数据
p  print     打印当前数据到屏幕上（格式为 type,x,y,z,rx,ry,rz 角度制）
q  quit      退出标定程序
```
```
[INFO] [1612856654.307437]: Get topic from param server: jaka_pose_topic:/jaka_pose camera_pose_topic:/aruco_single/pose
[INFO] [1612856655.311045]: Waiting jaka pose topic data ...
[INFO] [1612856656.313039]: Waiting jaka pose topic data ...
[INFO] [1612856657.314364]: Waiting jaka pose topic data ...
input:  r     record,c    calculate,s     save,q    quit:
```
拖拽机械臂或者用视校器移动机械臂，但要保证相机事业中依然可以看到标定板。
输入`r`记录一组手眼数据。

#### 5.生成参数
完成标定之后输入`s`即可进行保存，将保存标定结果数据和计算所使用的数据。

##### 标定结果正确与否的测试
观察数据计算结果的标准差大小。
   每次计算之后，程序都会输出不同算法下标定结果点的平均数、方差、标准差三项数值。
   
   > 由于标定过程中标定板是没有发生移动的，所以我们通过机械臂的末端位置、标定结果（手眼矩阵）、标记物在相机中的位姿即可计算出标定板在机器人基坐标系下的位姿，如果标定结果准确该位姿应该是没有变化的。
   
   可以比较最终数据的波动情况来判定标定结果的好坏。

比如：

标定板在机械臂基坐标系的位置1：
```
Tsai-Lenz               x            y             z            rx            ry           rz     distance
-----------  ------------  -----------  ------------  ------------  ------------  -----------  -----------
point0       -0.45432      0.0488783     0.000316595   0.0420852    -0.0245641    1.52064      0.456941
point1       -0.457722     0.054523      0.0121959    -0.0266793     0.0050922    1.53391      0.461119
point2       -0.457198     0.0535639     0.00246136    0.0252805    -0.0329136    1.51927      0.460331
point3       -0.453302     0.0618366     0.00165179    0.0405718    -0.0472311    1.53318      0.457503
point4       -0.455802     0.0589413     0.000377679   0.0222521    -0.0360589    1.51963      0.459598
point5       -0.455392     0.0615103     0.00584822    0.0365886    -0.033448     1.50684      0.459565
point6       -0.451144     0.0571198     0.00498852    0.0618337    -0.0170326    1.52463      0.454773
point7       -0.452829     0.0588266    -0.000827528   0.0324858    -0.0292652    1.52268      0.456635
point8       -0.454238     0.063634      0.00488078    0.0411648    -0.0373725    1.51611      0.458699
point9       -0.453579     0.0631788     0.00390939    0.0339742    -0.0645821    1.53168      0.457974
point10      -0.454952     0.066057     -0.00144969    0.0399135     0.0029201    1.5053       0.459725
point11      -0.459518     0.0553877    -0.00209946    0.0450864    -0.0147387    1.50702      0.462848
point12      -0.454928     0.0590754    -0.0045181     0.0297534    -0.0296122    1.52043      0.45877
point13      -0.455234     0.0527075    -0.00389213    0.0358822    -0.0260668    1.51244      0.458292
mean         -0.455011     0.0582314     0.0017031     0.0328709    -0.027491     1.51955      0.45877
var           4.21677e-06  2.16484e-05   1.84365e-05   0.000357231   0.000305579  8.29112e-05  3.79771e-06
std           0.00205348   0.00465279    0.00429378    0.0189005     0.0174808    0.00910556   0.00194877
```
标定板在机械臂基坐标系的位置2：
```
Tsai-Lenz              x            y            z           rx            ry           rz     distance
-----------  -----------  -----------  -----------  -----------  ------------  -----------  -----------
point0       -0.428394    0.052448     0.0353171    0.0259549    -0.0541487    1.57929      0.433035
point1       -0.427841    0.0448442    0.0345359    0.0454481    -0.0371304    1.55639      0.431569
point2       -0.424889    0.0486165    0.0278942    0.0455775    -0.0438353    1.57073      0.42857
point3       -0.421985    0.0485442    0.0311218    0.0138094    -0.0307286    1.55606      0.425906
point4       -0.428353    0.0454091    0.0326252    0.039192     -0.0492181    1.59177      0.431987
point5       -0.432111    0.0458869    0.0359774    0.04632      -0.0383476    1.55942      0.436028
mean         -0.427262    0.0476248    0.0329119    0.0360503    -0.0422348    1.56894      0.431183
var           9.9672e-06  6.79218e-06  7.71397e-06  0.000148499   6.11379e-05  0.000174299  1.03945e-05
std           0.00315709  0.00260618   0.0027774    0.012186      0.00781908   0.0132022    0.00322405
```

我们可以观察两次标定结果的距离的标准差，第一次的标准差小于的第二次的标准差，这表示第一次的标定结果好于第二次。
> 标准差越小，数据越聚集。

### 3、结合AUBO机械臂使用
#### 1.配置aubo机械臂ip地址信息

#### 2.配置运行aubo通信节点

#### 3.运行标定程序

#### 4.开始标定

#### 5.生成参数



### 4、使用系统ArUco获取标定板位姿
- 在线生成标定板:https://chev.me/arucogen/

#### 1.安装
  Kinetic：
  ```
  sudo apt-get install ros-kinetic-aruco*
  ```
  Melodic：
  ```
  sudo apt-get install ros-melodic-aruco*
  ```
  其他版本

  ```
  sudo apt-get install ros-版本名称-aruco*
  ```

#### 2.修改参数
可以直接使用本仓库中handeye-calib所提供的`aruco_start_usb_cam.launch`或者`aruco_start_realsense_sdk.launch` 分别使用realsense和usb相机的驱动来运行，这个视你的相机而定。

需要修改的参数如下：

- camera_info_url 相机标定文件所在位置
- video_device： 设备位置
- image_width： 图片宽度
- image_height： 图片高度
- markerId： 标定板编号，就是你所用的标定板的id，可以通过`在线生成标定板:https://chev.me/arucogen/`进行生成并打印
- markerSize：标定板的宽度 单位m
```

<arg name="camera_info_url"  default="file:///home/dev/.ros/camera_info/ost.yaml"/>
<arg name="video_device"     default="/dev/video2"/>
<arg name="image_width"      default="1280"/>
<arg name="image_height"     default="720"/>

<arg name="markerId"        default="0"/>
<arg name="markerSize"      default="0.151"/>    
```

#### 3.开始运行

```
source devel/setup.bash
roslaunch handeye-calib aruco_start_usb_cam.launch
```



## 其他
### 1.、使用ROS usb_cam驱动相机：
#### a.安装usbcam
Kinetic：
```
sudo apt-get install ros-kinetic-usb-cam
```
Melodic：
```
sudo apt-get install ros-melodic-usb-cam
```
其他版本

```
sudo apt-get install ros-melodic-版本名称-cam
```

#### b.修改launch文件

进入目录：
```
roscd usb_cam
cd launch
sudo gedit usb_cam-test.launch 
```
目前主要修改device和width两个参数，可以使用`ls /dev/video*`查看系统视频设备。
```
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <!-- modify the video device to your device -->
    <param name="video_device" value="/dev/video2" />
    <!-- modify the size of your device -->
    <param name="image_width" value="1280" />
    <param name="image_height" value="720" />
    <param name="pixel_format" value="yuyv" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" ou$
    <remap from="image" to="/usb_cam/image_raw"/>
    <param name="autosize" value="true" />
  </node>
</launch>
```
#### c. 启动相机

```
roslaunch usb_cam usb_cam-test.launch
```

### 2、使用ROS进行相机标定
#### 1.使用ROS自带的标定程序进行标定。

- 小工具：棋盘格pdf在线生成网站：[点击打开](https://calib.io/pages/camera-calibration-pattern-generator),生成后使用一比一打印要比手动量的要精准哦。
- 标定完成后点击Save可以保存标定所用的图片和参数矩阵。在终端里会输出标定产生的压缩包，默认放在`/tmp`目录下。

##### a.运行标定程序

运行前需要根据你的棋盘格修改两个参数，一个是size参数为棋盘格角点数量比如8x9=72个格子的棋盘格，角点个数为7x8=63个，size参数就要写7x8。另外一个参数为square，传入的参数为棋盘格一个小格子的宽度（注意单位为m）。
```
rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.015 image:=/usb_cam/image_raw camera:=/usb_cam
```
##### b.生成标定文件  
标定完成后点击Calculate会稍微有点卡顿，不要担心后台正在进行标定，完成后下面的SAVE和COMMIT按钮变为可用状态，点击SAVE即可保存标定完成后的文件。

##### c.在ROS中使用该参数
可以在usb_cam的launch文件中增加以下参数，重新启动usb_cam节点，即可使用该标定参数。

```
<param name="camera_info_url" type="string" value="file:///home/dev/.ros/camera_info/ost.yaml"/>
```

## 版本日志

- V2.1
  添加Aruco 启动文件。

- V2.0
  添加配合aubo机械臂进行手眼标定程序。

- V1.5
  添加配合jaka机械臂进行手眼标定程序。

- V1.0
  完成基础标定程序包，可以通过文件输出位姿进行，输出标定结果。并进行校验。

## 贡献

- [@sangxin](sangxin@infore.com) InforeRobot
- [@duanxh](duanxh@infore.com) InforeRobot

## 参考
- easy-handeye
- opencv-calibHandEye
- jaka&&aubo
- aruco_ros
- [csdn](https://blog.csdn.net/sandy_wym_/article/details/83996479)