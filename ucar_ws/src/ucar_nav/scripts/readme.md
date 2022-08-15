# scripts文件夹的说明：本文件夹主要存放一些带有整合性质的代码，主要是将比赛的任务都整合到一个程序里面，接下来是一些文件夹的介绍
- _pycache_       : 是一些python程序的二进制编译文件
- dynamic_param   : 是对于每一段路径的针对性参数的存放位置
- final           : 之前用于存放最终决赛的文件，现在处于弃用状态
- main_version    : 存放一些main程序的历史版本，每个版本的介绍可详见main_version中的readme.md文档
- pose            : 存放地图点位的文件夹，其中odom坐标为xyz坐标系表示，yaw用四元数表示，为绕z轴旋转，
                    对于四元数的转换，可以利用这个网站 https://quaternions.online/
- interactive.py  : 本模块主要用于实现用户交互功能和记录功能，将输出信息全部保存到 LOG_FILE 文件中，关键信息单独保存到 VITAL_LOG_FILE
- log             : 记录终端信息的文本
- main~~.py       : 整合任务的启动文件与训练文件
- mbot_teleop.py  : 键盘控制节点
- pid.py          : PID控制的类文件
- pid_server.py   : PID控制节点的文件
- pid_servo.py    : 北京邮电大学的PID控制节点（当初用于参考的）
- ros_module.py   : 讯飞官方的导航类文件
- ros_module_x.py : 针对所需功能进行优化的导航类文件
- set_pointer.py  : 北邮所用的节点文件
- test.py         : 功能测试文件（具体我也不懂它想干啥）

# 日常的代码与调试记录

## 21.11.21 林锐东整理了一些main程序，建立了一个main文件夹，并对每个程序做出了解释
## 21.11.21 林锐东将pose点放置于/scripts/poses文件夹中了
## 21.11.21 林锐东将加载的动态参数param放置于/scripts/dynamic_param文件夹中了
## 21.11.21 林锐东发现，如果出发点不一样的时候，那么rviz上现实的坐标和odom底盘现实的坐标是不太一样的，所以这时候的坐标需要使用 '' 'rostopic echo /odom' '' 指令进行坐标的查询。综上所述，我们在比赛所用的冲刺点的坐标与 main_train3.py 程序里所使用的冲刺点坐标有所不同，这一点作者也会在函数代码的注释里体现。