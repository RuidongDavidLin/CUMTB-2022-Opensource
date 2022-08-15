[TOC]

## 1. 全部功能

### 1.1 里程计与底盘的坐标变换是否发布由参数实时配置
参数名称：`publish_odom_tf`
1. 设置为true:   发布 odom -> base_link的坐标变换
2. 设置为false: 不发布 odom -> base_link的坐标变换

> 注，以下运动相关服务优先度大于实时运动控制（cmd_vel）当服务完成后才会继续执行实时控制。（手柄的优先级更高，不过不涉及web接口，此处不详细描述。手柄 > 运动服务 > cmd_vel）

### 1.2 停止运动服务
服务名：`stop_move` （基于速度、距离、角度的运动）
用途：调用后立即终止正在运行的`time_move`或`vel_dist_move`。
并回到cmd控制模式
服务数据类型：std_srvs/Trigger.srv
```
---
bool success                  #运动结束时返回true，异常返回false
string message                #返回的字符串信息
```
返回枚举：
1. 成功停止
```
res.success = true;
res.message = "Move stopped ";
```
2. 不在move模式，不需要停止
```
res.success = false;
res.message = "Not in MOTOR_MODE_MOVE.";
```

### 1.3 实时控制
方式：发布话题。
话题名：`cmd_vel`
msg类型：geometry_msgs/Twist.msg
```
geometry_msgs/Vector3 linear
  float64 x                   # x方向线速度
  float64 y                   # y方向线速度
  float64 z                   # 无效
geometry_msgs/Vector3 angular
  float64 x                   # 无效
  float64 y                   # 无效
  float64 z                   # z轴角速度
```

### 1.4 设置灯光效果
服务名：`get_battery_state`
服务数据类型：aicar_controller/GetBatteryInfo.srv
```
uint8 MODE_NORMAL = 0 # 常亮
uint8 MODE_BLINK  = 1 # 闪烁
uint8 MODE_BREATH = 2 # 呼吸

uint8   mode_type   # 灯光模式
float64 frequency   # 模式频率（常亮模式自动忽略）
uint8 red_value   # 红色光亮度范围（0-255）
uint8 green_value # 绿色光亮度范围（0-255）
uint8 blue_value  # 蓝色光亮度范围（0-255）
---
bool success        # 设置是否成功
string message      # 异常log信息
```
response枚举：
1、设置失败（可能与MUC通信异常）
  res.success = false;
  res.message = "Can't connect base's MCU.";
2、设置成功
  res.success = true;
  res.message = "Set LED success.";

### 1.5 底盘电池信息发布（仅电池剩余电量百分比有效，即：percentage）
话题名：`battery_state`
`sensor_msgs/BatteryState.msg`
```
POWER_SUPPLY_STATUS_UNKNOWN      = 0            # 未知
POWER_SUPPLY_STATUS_CHARGING     = 1            # 充电
POWER_SUPPLY_STATUS_DISCHARGING  = 2            # 放电中
POWER_SUPPLY_STATUS_NOT_CHARGING = 3            # 未充电
POWER_SUPPLY_STATUS_FULL         = 4            # 满电
POWER_SUPPLY_HEALTH_UNKNOWN               = 0   # 未知
POWER_SUPPLY_HEALTH_GOOD                  = 1   # 良好
POWER_SUPPLY_HEALTH_OVERHEAT              = 2   # 过热
POWER_SUPPLY_HEALTH_DEAD                  = 3   # 没电
POWER_SUPPLY_HEALTH_OVERVOLTAGE           = 4   # 过电压
POWER_SUPPLY_HEALTH_UNSPEC_FAILURE        = 5   # 未知错误
POWER_SUPPLY_HEALTH_COLD                  = 6   # 过冷
POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7   # 
POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE   = 8   # 
POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0   # 未知
POWER_SUPPLY_TECHNOLOGY_NIMH    = 1   # Ni-MH  battery 镍氢电池
POWER_SUPPLY_TECHNOLOGY_LION    = 2   # Li-ion battery 锂离子电池
POWER_SUPPLY_TECHNOLOGY_LIPO    = 3   # Li-Po  battery 锂聚合物电池
POWER_SUPPLY_TECHNOLOGY_LIFE    = 4   # li-Fe  battery 锂铁电池
POWER_SUPPLY_TECHNOLOGY_NICD    = 5   # NiCd   battery 镍镉电池
POWER_SUPPLY_TECHNOLOGY_LIMN    = 6   # LiMn   battery 锂锰电池

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 voltage                      # Voltage in Volts (Mandatory)
float32 current                      # Negative when discharging (A)  (If unmeasured NaN)
float32 charge                       # Current charge in Ah  (If unmeasured NaN)
float32 capacity                     # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity              # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage                   # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number
```
当前版本使用到的status数据：
```cpp
  battery_msg.power_supply_status     = 2;           // 放电中 即正在运行
  battery_msg.power_supply_health     = 1;           // 良好
  battery_msg.power_supply_technology = 0;           // UNKNOWN
  battery_msg.present    = true;                     // 存在电池
  battery_msg.percentage = current_battery_percent_; // 电池电量百分比
```

### 1.6 获取底盘电量（仅电池剩余电量百分比有效，即：percentage）
服务名：`get_battery_state`
用途： 调用主动获取电池状态。
服务数据类型：aicar_controller/GetBatteryInfo.srv
```
---
sensor_msgs/BatteryState battery_state
```
即，服务的request为空，response为上面话题发布的电池状态。

response枚举：
  1、获取异常(可能是串口未与MCU通讯)：
  ```
  response.battery_state.percentage              = -1 # 表示异常
  response.battery_state.power_supply_status     =  0 # 未知
  response.battery_state.power_supply_health     =  0 # 未知
  response.battery_state.power_supply_technology =  0 # 未知
  ```

  2、正常获取：
  ```
  response.battery_state.percentage              = <current_percent> # 电池剩余电量百分比(0-100)。
  response.battery_state.power_supply_status     =  2 # 放电中
  response.battery_state.power_supply_health     =  1 # 良好
  response.battery_state.power_supply_technology =  0 # 未知
  ```

### 1.7.获取当前最大速度
服务名：`get_max_vel`
服务数据类型：GetMaxVel.srv
```
---
float64 max_linear_velocity   # 底盘最大线速度 (m)
float64 max_angular_velocity  # 底盘最大角速度 (rad/s)
```

### 1.8 设置底盘最大速度
服务名：`set_max_vel`
服务数据类型：SetMaxVel.srv
```
float64 max_linear_velocity   # 底盘最大线速度 (m)
float64 max_angular_velocity  # 底盘最大角速度 (rad/s)
---
bool success
string message
```
注：调用后同时会更新参数服务器中参数`/base_driver/linear_speed_max`、`/base_driver/angular_speed_max`.
也可以通过这两个参数获取当前最大速度。

> 以下两个是 tf_server提供的

### 1.9 获取底盘运行总里程
方式：获取参数（get_param）
参数名：`Mileage_sum`
单位：米。（即记录了该底盘自出厂的运行里程）


### 1.10 设置camera、lidar、imu的位姿 （tf_server）
方式：call server
服务名：`set_camera_tf`、`set_lidar_tf`、`set_imu_tf`
服务数据类型：SetSensorTF.srv
```
float64  pose_x
float64  pose_y
float64  pose_z
float64  euler_r
float64  euler_p
float64  euler_y
---
bool success
string message
```

### 1.11 获取camera、lidar、imu的位姿（tf_server）
方式：call server
服务名：`get_camera_tf`、`get_lidar_tf`、`get_imu_tf`
服务数据类型：GetSensorTF.srv
```
---
float64  pose_x
float64  pose_y
float64  pose_z
float64  euler_r
float64  euler_p
float64  euler_y
```