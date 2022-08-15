# 智能驾驶小车ros示例代码介绍
[TOC]
## 简介
该示例代码包含整个ROS的workspace。用`catkin_make`编译即可使用。
建议放在 `~/` 目录下。
>前提条件：本工作空间需要已经完成ROS的安装、OpenCV的编译。

以在 `~/` 为例：
```bash
cd ~/ucar_ws
catkin_make
```
## 文件结构
本workspace目录结构如下：
```bash
.
├── startup_scripts     # 设备权限设施脚本，用于在`/etc/udev/rules.d/`中添加ucar.rules
├── ucar_controller     # 智能驾驶小车的底盘控制ros包(mini版本集成了imu)
├── fdilink_ahrs        # imu示例ros包（晓版本用imu需要单独调用本包）
├── ydlidar             # 激光雷达示例ros包
├── ucar_map            # 建图示例ros包
├── ucar_nav            # 导航示例ros包
├── car_race            # 智能车比赛示例代码包
├── xf_mic_asr_offline  # 麦克风阵列示例ros包
├── geometry            # tf2相关包
├── geometry2           # tf2相关包
└── CMakeLists.txt
```
1. `startup_scripts`：文件夹存放了快速配置小车外设权限的脚本。两个脚本分别对应两个版本小车。
2. `ucar_controller`：是控制小车底盘的驱动包，实现了对4个电机、彩色LED、IMU的控制。
3. `ydlidar`：激光雷达的驱动包。
4. `ucar_map`：基于Cartographer算法的SLAM建图的配置示例ros包。
5. `ucar_nav`：基于ROS move_base的导航配置示例ros包。
6. `ucar_race`：智能车竞赛参考代码。
7. `xf_mic_asr_offline`：麦克风阵列驱动包。
8. `geometry` 与 `geometry2` tf2相关包。
具体使用见下文以及相应包中的`README.md`文件。
>没有包含的usb摄像头可以参考[usb_cam](https://github.com/ros-drivers/usb_cam)包来使用。下面提到的`startup_scripts`已经将摄像头设备链接为`/dev/ucar_video`。

## 各包说明
### 1. startup_scripts
`initdev_mini.sh` 与 `initdev_xiao.sh` 分别对应晓mini版本的智能驾驶小车和晓版本的智能驾驶小车。
```bash
cd ./ucar_ws/src/startup_scripts/
sudo chmod 777 ./initdev*.sh
sudo ./initdev_mini.sh # 对应晓mini版本，晓版本勿执行这句
sudo ./initdev_xiao.sh # 对应晓版本，晓mini版本勿执行这句
sudo reboot # 可选，最好完成一次重启
```
这个脚本完成了`/etc/udev/rules.d/ucar.rules`文件的创建与udev服务重启。
以及，在`~/.bashrc`中添加了对`ucar_ws`的ROS环境的source。

### 2. ucar_controller
#### 2.1 简介
`ucar_controller`作为与下位机mcu通讯的ROS包实现了电机控制、灯光控制、mcu板载imu数据读取等功能。我们可以通过发布`/cmd_vel(geometry_msgs/Twist)`速度控制话题，指定xy方向的平移速度以及yaw角的旋转速度来控制小车运动。同时发布里程计话题`/odom(nav_msgs/Odometry)`实时发布小车里程计记录的速度、位置。其他功能阅读`ucar_controller`包中的`readme.md`文档。
#### 2.2 所需安装依赖
```bash
sudo apt update
sudo apt install ros-melodic-serial libeigen3-dev
sudo pip3 uninstall em
sudo pip3 install empy -i https://pypi.tuna.tsinghua.edu.cn/simple
```
编译：
```bash
cd ~/ROS/ucar_ws
catkin_make
```
#### 2.3 简单验证
1、启动节点：
```bash
roslaunch ucar_controller base_driver.launch
```
2、发布话题控制：
`base_driver`节点订阅`/cmd_vel`话题，msg类型为`geometry_msgs/Twist`。
我们可以通过发布`/cmd_vel`话题来控制小车移动，比如：
<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>重新开启一个终端。
用`rostopic`工具发布话题。（注意用<kbd>Tab</kbd>键自动补全来填写`geometry_msgs/Twist`数据，不然手动输入容易出错）
```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 1.0"
```
输入指令后小车会持续逆时针自转。

### 3. fdilink_ahrs
#### 3.1 简介
`fdilink_ahrs`作为与晓版本小车nano板载imu通讯的ROS包实现了imu数据读取发布的功能。我们可以通过订阅`/imu(sensor_msgs/Imu)`话题来获取imu数据。详细功能阅读`fdilink_ahrs`包中的`README.md`文档。
#### 3.2 启动
完成编译后使用如下指令启动该节点：
```bash
roslaunch fdilink_ahrs ahrs_driver.launch
```

### 4. ydlidar
#### 4.1 简介
激光雷达的驱动包，直接编译即可使用。默认发布`/scan`话题的数据。对应的数据类型为：`sensor_msgs/LaserScan`。
#### 4.2 启动与配置
完成编译后使用如下指令启动该节点：
```bash
roslaunch ydlidar ydlidar.launch
```
对应的`launch`文件保存在：`~/ucar_ws/src/ydlidar/launch/ydlidar.launch`。
修改`ydlidar.launch`文件中的参数可以对`topic_name`、`frame_id`等参数进行修改。

### 5. ucar_nav
#### 5.1 简介
`ucar_nav`包基于`navigation`是包集，完成了基于`move_base`的导航配置，供参考使用。
#### 5.2 安装
新打开一个终端，输入以下命令安装
```bash
sudo apt-get install ros-melodic-navigation*
```
#### 5.3 使用
打开新的终端,运行下面命令
```bash
roslaunch ucar_nav ucar_navigation.launch
```
`ucar_nav/launch/`文件夹中的navigation.launch为导航的启动文件。`ucar_nav/launch/config/`文件夹主要用于存放导航所需要的yaml文件,其中`ucar_nav/launch/config/amcl/`中是用于定位的参数文件，`ucar_nav/launch/config/move_base/`是存放着路径规划和感知地图的参数文件。

#### 编译过程中可能出现的错误
1、No module named 'em'
解决方法：打开一个新的终端执行下面的命令
```bash
sudo pip3 uninstall em
sudo pip3 install empy -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 6. ucar_map
#### 6.1 简介
SLAM建图示例包，本包配置了Google的Cartographer的2D建图启动文件。
#### 6.2 cartographer安装
```bash
sudo apt-get install ros-melodic-cartographer
sudo apt-get install ros-melodic-cartographer-ros
```
#### 6.2 启动建图
##### 6.2.1 分步启动建图
打开3个新的终端,运行下面命令：
```bash
roslaunch ucar_controller base_driver.launch
roslaunch ydlidar ydlidar.launch
roslaunch ucar_map cartographer_start.launch
```
##### 6.2.2 一键启动建图
```bash
roslaunch ucar_map ucar_mapping.launch
```
`ucar_mapping.launch`中包含了分布启动建图中的3个launch文件
>说明:
上文已经介绍了
base_driver.launch是启动小车底盘控制，ydlidar.launch是启动雷达
cartographer_start.launch是单独启动cartographer_ros。



### 7. xf_mic_asr_offline
#### 7.1 简介
麦克风阵列ROS驱动示例包，本包基于ROS实现了上传音频、获取唤醒角度、设置灯光、设置主麦设置唤醒词等功能。
详细内容阅读：`ucar_ws/src/xf_mic_asr_offline/doc/麦克风阵列使用说明v1.9.x.pdf`


# 修改记录
20210323：新增fdilink_ahrs软件包的简介
20210309：第一版
20210304：初次编写
