#include <ucar_controller/base_driver.h>
#include <Eigen/Eigen>
namespace ucarController
{
baseBringup::baseBringup() :x_(0), y_(0), th_(0)
{
  ros::NodeHandle pravite_nh("~");
  pravite_nh.param("provide_odom_tf", provide_odom_tf_,true);
  pravite_nh.param("vel_topic", vel_topic_,std::string("/cmd_vel"));///smooth_cmd_vel
  
  pravite_nh.param("joy_topic",  joy_topic_, std::string("/joy"));
  pravite_nh.param("odom_topic", odom_topic_,std::string("/odom"));
  pravite_nh.param("battery_topic", battery_topic_,std::string("/battery_state"));
  //serial
  pravite_nh.param("port", port_, std::string("/dev/base_serial_port"));
  pravite_nh.param("baud", baud_, 115200);                
  pravite_nh.param("serial_timeout", serial_timeout_, 50);//ms
  pravite_nh.param("rate", rate_, 20);                    //hz
  pravite_nh.param("duration", duration_, 0.01);
  pravite_nh.param("cmd_timeout", cmd_dt_threshold_, 0.2);
  
  pravite_nh.param("base_frame", base_frame_, std::string("base_footprint"));
  pravite_nh.param("odom_frame", odom_frame_, std::string("odom"));

  pravite_nh.param("encode_resolution", encode_resolution_, 270);  //   
  pravite_nh.param("wheel_radius", wheel_radius_, 0.04657);  //   m
  pravite_nh.param("period", period_, 50.0); //ms
  pravite_nh.param("base_shape_a", base_shape_a_, 0.2169);  //   m
  pravite_nh.param("base_shape_b", base_shape_b_, 0.0);  //   m

  pravite_nh.param("linear_speed_max",   linear_speed_max_, 3.0);  //   m/s
  pravite_nh.param("angular_speed_max", angular_speed_max_, 3.14);// rad/s
  pravite_nh.setParam("linear_speed_max" ,linear_speed_max_);
  pravite_nh.setParam("angular_speed_max",angular_speed_max_);

  pravite_nh.param("Mileage_file_name", Mileage_file_name_, std::string("car_Mileage_info.txt"));//
  Mileage_file_name_        = ros::package::getPath("ucar_controller") + std::string("/log_info/") + Mileage_file_name_;
  Mileage_backup_file_name_ = Mileage_file_name_ + ".bp";
  pravite_nh.param("debug_log", debug_log_, true);//  true rosinfo 等等  打印log数据

  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu")); 
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
  read_first_ = false;
  imu_frist_sn_ = false;
  controll_type_ = MOTOR_MODE_CMD; // 1:vel_mode 0:joy_node
  linear_gain_   = 0.3;
  twist_gain_    = 0.7;
  linear_speed_min_  = 0;
  angular_speed_min_ = 0;
  current_battery_percent_ = -1;
  led_mode_type_   = 0;
  led_frequency_   = 0;
  led_red_value_   = 0;
  led_green_value_ = 0;
  led_blue_value_  = 0;

  getMileage();

  odom_pub_    = nh_.advertise<nav_msgs::Odometry>(odom_topic_.c_str(),10);
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>(battery_topic_.c_str(),10);
  vel_sub_     = nh_.subscribe<geometry_msgs::Twist>(vel_topic_.c_str(), 1, &baseBringup::velCallback,this);
  joy_sub_     = nh_.subscribe<sensor_msgs::Joy>(joy_topic_.c_str(),     1, &baseBringup::joyCallback,this);
  imu_pub_     = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10);
  mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10);

  stop_move_server_   = nh_.advertiseService("stop_move", &baseBringup::stopMoveCB, this);
  set_max_vel_server_ = nh_.advertiseService("set_max_vel", &baseBringup::setMaxVelCB, this);
  get_max_vel_server_ = nh_.advertiseService("get_max_vel", &baseBringup::getMaxVelCB, this);
  get_battery_state_server_ = nh_.advertiseService("get_battery_state", &baseBringup::getBatteryStateCB, this);
  set_led_server_     = nh_.advertiseService("set_led_light", &baseBringup::setLEDCallBack, this);
  
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    serial_.open(); 
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port "); 
    exit(0); 
  }
  if(serial_.isOpen()) 
  { 
    ROS_INFO_STREAM("Serial Port initialized"); 
  }else{ 
    ROS_ERROR_STREAM("Unable to initial Serial port "); 
    exit(0);
  } 
  current_time_ = ros::Time::now();
  last_time_    = ros::Time::now();
  setSerial();
  openSerial();
  writeThread_   = new boost::thread(boost::bind(&baseBringup::writeLoop,   this));
  processThread_ = new boost::thread(boost::bind(&baseBringup::processLoop, this));

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("ucarController Ready!");
  ros::waitForShutdown();
}

void baseBringup::setSerial()
{
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    // serial_.open(); 
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("AIcarController setSerial failed, try again!");
    ROS_ERROR("AIcarController setSerial: %s", e.what());
    setSerial();
  }
  catch (...)
  {
    ROS_ERROR("AIcarController setSerial failed with unknow reason, try again!");
    setSerial();
  }
  ros::Duration(cmd_dt_threshold_).sleep();
}

void baseBringup::openSerial()
{
  bool first_open = true;
	while(ros::ok())
  {
    try
    {
      if(serial_.isOpen()==1)
      {
        ROS_INFO("AIcarController serial port open success.\n");
        return;
      }
      else
      {
        if (first_open){
          ROS_INFO("AIcarController openSerial: start open serial port\n");
        } 
        serial_.open();
      }
    }
    catch(const std::exception& e)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: %s", e.what());
        ROS_ERROR("AIcarController openSerial: unable to open port, keep trying\n");
        // std::cerr << e.what() << '\n';
      }
    }
    catch(...)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: unable to open port with unknow reason, keep trying\n");
      }
    }
    first_open = false;
    // sleep(1);
    ros::Duration(cmd_dt_threshold_).sleep();
  }
}//openSerial

void baseBringup::writeLoop()
{
  ROS_INFO("baseBringup::writeLoop: start");
  led_timer = 0;
  ros::Rate loop_rate(rate_);
  while(ros::ok())
  {
    try
    {
      boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
      int cur_controll_type = controll_type_;
      lock.unlock();
      double linear_x;
      double linear_y;
      double angular_z;
      switch (cur_controll_type)
      {
        case MOTOR_MODE_JOY:{
          lock.lock();
          linear_x  = joy_linear_x_;
          linear_y  = joy_linear_y_;
          angular_z = joy_angular_z_;
          lock.unlock();
          break;
        }
        case MOTOR_MODE_CMD:{
          lock.lock();
          double dt = (ros::Time::now() - last_cmd_time_).toSec();
          if (dt > cmd_dt_threshold_)
          {
            linear_x  = 0;
            linear_y  = 0;
            angular_z = 0;
          }        
          else
          {
            linear_x  = cmd_linear_x_;
            linear_y  = cmd_linear_y_;
            angular_z = cmd_angular_z_;
          }
          lock.unlock();
          break;
        }
        case MOTOR_MODE_MOVE:{
          lock.lock();
          linear_x  = move_linear_x_;
          linear_y  = move_linear_y_;
          angular_z = move_angular_z_;
          lock.unlock();
          break;
        }
        default:
          ROS_ERROR("base_driver-writeLoop: controll_type_ error!");
          break;
      }
      if(linear_x > linear_speed_max_)
        linear_x = linear_speed_max_;
      else if(linear_x < -linear_speed_max_)
        linear_x = -linear_speed_max_;
      
      if(linear_y > linear_speed_max_)
        linear_y = linear_speed_max_;
      else if(linear_y < -linear_speed_max_)
        linear_y = -linear_speed_max_;

      if(angular_z > angular_speed_max_)
        angular_z = angular_speed_max_;
      else if(angular_z < -angular_speed_max_)
        angular_z = -angular_speed_max_;

      double vw1 = linear_x - linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      double vw2 = linear_x + linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw3 = linear_x - linear_y + angular_z* (base_shape_a_ + base_shape_b_);
      double vw4 = linear_x + linear_y - angular_z* (base_shape_a_ + base_shape_b_);
      lock.lock();
      pack_write_.write_tmp[0] = 0x63;
      pack_write_.write_tmp[1] = 0x75;
      pack_write_.pack.ver     = 0;                 // 0x00  version
      pack_write_.pack.len     = 11; // motor + led 
      // pack_write_.pack.sn_num  = write_sn_++;
      pack_write_.pack.data.pluse_w1 = -period_/1000.0*vw1*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w2 =  period_/1000.0*vw2*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w3 = -period_/1000.0*vw4*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w4 =  period_/1000.0*vw3*(encode_resolution_/(2.0*Pi*wheel_radius_));
      //LED value
      int cur_led_mode = led_mode_type_;

      led_timer++;
      lock.unlock();
      switch (cur_led_mode)
      {
        // case LED_MODE_NORMAL:
        case ucar_controller::SetLEDMode::Request::MODE_NORMAL : 
        {
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }
        case ucar_controller::SetLEDMode::Request::MODE_BLINK:
        {
          double t = (double)led_timer/(double)rate_;
          lock.lock();
          // double t = (int)ros::Time::now().toSec();
          double f = led_frequency_;
          int blink = (int)(2.0*t*f)%2;
          pack_write_.pack.red_value   = led_red_value_   * blink;
          pack_write_.pack.green_value = led_green_value_ * blink;
          pack_write_.pack.blue_value  = led_blue_value_  * blink;
          lock.unlock();
          break;
        }
        case ucar_controller::SetLEDMode::Request::MODE_BREATH:
        {
          double t = ros::Time::now().toSec();
          lock.lock();
          // double t = (double)led_timer/(double)rate_;
          double w = 2 * Pi * led_frequency_;
          pack_write_.pack.red_value   = 0.5 * (led_red_value_   + led_red_value_   * sin(w * t));
          pack_write_.pack.green_value = 0.5 * (led_green_value_ + led_green_value_ * sin(w * t));
          pack_write_.pack.blue_value  = 0.5 * (led_blue_value_  + led_blue_value_  * sin(w * t));
          lock.unlock();
          break;
        }
        default:{
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }
      }
      lock.lock();
      setWriteCS(WRITE_MSG_LONGTH);
      size_t pack_write_s = serial_.write(pack_write_.write_tmp,WRITE_MSG_LONGTH);
      lock.unlock();
      if(debug_log_){
        cout << "write buf:" << endl;
        for (size_t i = 0; i < WRITE_MSG_LONGTH; i++)
        {
          cout << std::hex << (int)pack_write_.write_tmp[i] << " ";
        }
        cout << std::dec << endl;
      }
      loop_rate.sleep();
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("AIcarController writeLoop: %s\n", e.what());
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
  }  
}

baseBringup::~baseBringup()
{
  if( serial_.isOpen() )   
    serial_.close();
}

bool baseBringup::getBatteryStateCB(ucar_controller::GetBatteryInfo::Request &req,
                                    ucar_controller::GetBatteryInfo::Response &res)
{
  if (current_battery_percent_ == -1)  //初始值为 -1. 即没有获取到电量
  {
    res.battery_state.power_supply_status     = 0; // UNKNOWN
    res.battery_state.power_supply_health     = 0; // UNKNOWN
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_;
    lock.unlock();
    return true;
  }
  else
  {
    res.battery_state.power_supply_status     = 2; // 放电中 即正在运行
    res.battery_state.power_supply_health     = 1; // 良好
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    res.battery_state.present    = true;           // 存在电池
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_; // 电池电量百分比
    lock.unlock(); 
    return true;
  }
}

bool baseBringup::setLEDCallBack(ucar_controller::SetLEDMode::Request &req, 
                                 ucar_controller::SetLEDMode::Response &res)
{
  if (!read_first_)
  {
    res.success = false;
    res.message = "Can't connect base's MCU."; 
    return true;
  }
  else
  {
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    led_mode_type_   = req.mode_type;
    led_frequency_   = req.frequency;
    led_red_value_   = req.red_value;
    led_green_value_ = req.green_value;
    led_blue_value_  = req.blue_value;
    led_t_0 = ros::Time::now().toSec();
    led_timer = 0;
    lock.unlock();
    res.success = true;
    res.message = "Set LED success.";
    return true;
  }
}


bool baseBringup::getMaxVelCB(ucar_controller::GetMaxVel::Request &req, ucar_controller::GetMaxVel::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  res.max_linear_velocity  = linear_speed_max_ ;
  res.max_angular_velocity = angular_speed_max_;
  lock.unlock();
  return true;
}
bool baseBringup::setMaxVelCB(ucar_controller::SetMaxVel::Request &req, ucar_controller::SetMaxVel::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  linear_speed_max_  = req.max_linear_velocity ;
  angular_speed_max_ = req.max_angular_velocity;
  ros::NodeHandle pravite_nh("~");
  pravite_nh.setParam("linear_speed_max",linear_speed_max_);
  pravite_nh.setParam("angular_speed_max",angular_speed_max_);
  lock.unlock();
  if (linear_speed_max_ == req.max_linear_velocity && angular_speed_max_ == req.max_angular_velocity)
  {
    res.success = true;
    res.message = "Max vel set successfully.";
    return true;
  }
  res.success = false;
  res.message = "Max vel set faild.";
  return true;
}

bool baseBringup::stopMoveCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  //判断当前控制类型
  if (controll_type_ != MOTOR_MODE_MOVE){
    res.success = false;
    res.message = "Not in MOTOR_MODE_MOVE.";
    lock.unlock();
    return true;
  }
  else{
    move_linear_x_  = 0;
    move_linear_y_  = 0;
    move_angular_z_ = 0;
    res.success = true;
    res.message = "Move stopped ";
    controll_type_ = MOTOR_MODE_CMD;
    lock.unlock();
    return true;
  }
}

void baseBringup::processLoop()
{
  ROS_INFO("baseBringup::processLoop: start");
  uint8_t check_head_last[1]    = {0xFF};
  uint8_t check_head_current[1] = {0xFF};
  while(ros::ok()){
    if (!serial_.isOpen())
    {
      ROS_ERROR("serial unopen");
    }
    try
    {
      int head_type = 0;
      while(ros::ok()) 
      {                        
        size_t head_s = serial_.read(check_head_current,1);
        if (check_head_last[0] == 0x63 && check_head_current[0] == 0x76)
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
          pack_read_.read_msg.head[0] = check_head_last[0];
          pack_read_.read_msg.head[1] = check_head_current[0];
          check_head_last[0] = 0xFF;
          lock.unlock();
          head_type = 1; // base
          break;
        }
        else if (check_head_last[0] == 0xfc && (check_head_current[0] == 0x40 || check_head_current[0] == 0x41 || head_type == TYPE_INSGPS || 
                                                check_head_current[0] == TYPE_GROUND || check_head_current[0] == 0x50))
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
          if      (check_head_current[0] == 0x40)
          {
            imu_frame_.frame.header.header_start = 0xfc;
            imu_frame_.frame.header.data_type    = 0x40;
            head_type = 0x40;
          }
          else if (check_head_current[0] == 0x41)
          {
            ahrs_frame_.frame.header.header_start = 0xfc;
            ahrs_frame_.frame.header.data_type    = 0x41;
            head_type = 0x41;
          }
          else if (check_head_current[0] == TYPE_GROUND){
            head_type = TYPE_GROUND;
          }
          else if (check_head_current[0] == 0x50)
          {
            head_type = 0x50;
          }
          check_head_last[0] = 0xFF;
          lock.unlock();
          if(debug_log_){
            cout << "head_type: " << head_type << endl;
          }
          break;
        }
        check_head_last[0] = check_head_current[0];
      }
      if (head_type == 1)
      {
        size_t res = serial_.read(pack_read_.read_msg.read_msg,READ_DATA_LONGTH);
        if(debug_log_){
          cout << "serial_read: " <<endl;
          for (size_t i = 0; i < READ_MSG_LONGTH; i++)
          {
            cout << std::hex << (int)pack_read_.read_tmp[i] << " ";
          }
          cout << std::dec << endl;
        }
        if(!checkCS(READ_MSG_LONGTH))
        {
          ROS_WARN("check cs error");
        }
        else
        {
          processBattery();
          processOdometry();
        }
        if(!read_first_)
        {
          read_first_ = true;
        }
      }
      else if (head_type == 0x40 || head_type == 0x41|| head_type == TYPE_GROUND || head_type == 0x50 || head_type == TYPE_INSGPS)
      {
        processIMU(head_type);
      }
      else
      {
        if(debug_log_)
        {
          ROS_DEBUG("head_type ERROR.");
        }
      }
    }// try end
    catch(const std::exception& e)
    {
      ROS_ERROR("AIcarController readLoop: %s\n", e.what());
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
  }
}

void baseBringup::processIMU(uint8_t head_type)
{
  uint8_t check_len[1] = {0xff};
  size_t len_s = serial_.read(check_len, 1);
  if (debug_log_){
    std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
  }
  if (head_type == TYPE_IMU && check_len[0] != IMU_LEN)
  {
    ROS_WARN("head_len error (imu)");
    return;
  }else if (head_type == TYPE_AHRS && check_len[0] != AHRS_LEN)
  {
    ROS_WARN("head_len error (ahrs)");
    return;
  }else if (head_type == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
  {
    ROS_WARN("head_len error (insgps)");
    return;
  }
  else if (head_type == TYPE_GROUND || head_type == 0x50) // 无效数据，防止记录失败
  {
    uint8_t ground_sn[1];
    size_t ground_sn_s = serial_.read(ground_sn, 1);
    if (++read_sn_ != ground_sn[0])
    {
      if ( ground_sn[0] < read_sn_)
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_1.");
        }
        sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
        read_sn_ = ground_sn[0];
      }
      else
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_2.");
        }
        sn_lost_ += (int)(ground_sn[0] - read_sn_);
        read_sn_ = ground_sn[0];
      }
    }
    uint8_t ground_ignore[500];
    size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
    return;
  }
  //read head sn 
  uint8_t check_sn[1] = {0xff};
  size_t sn_s = serial_.read(check_sn, 1);
  uint8_t head_crc8[1] = {0xff};
  size_t crc8_s = serial_.read(head_crc8, 1);
  uint8_t head_crc16_H[1] = {0xff};
  uint8_t head_crc16_L[1] = {0xff};
  size_t crc16_H_s = serial_.read(head_crc16_H, 1);
  size_t crc16_L_s = serial_.read(head_crc16_L, 1);
  if (debug_log_){
    std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
    std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
    std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
    std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
  }
  // put header & check crc8 & count sn lost
  if (head_type == TYPE_IMU)
  {
    imu_frame_.frame.header.data_size      = check_len[0];
    imu_frame_.frame.header.serial_num     = check_sn[0];
    imu_frame_.frame.header.header_crc8    = head_crc8[0];
    imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
    if (CRC8 != imu_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = imu_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    baseBringup::checkSN(TYPE_IMU);
  }
  else if (head_type == TYPE_AHRS)
  {
    ahrs_frame_.frame.header.data_size      = check_len[0];
    ahrs_frame_.frame.header.serial_num     = check_sn[0];
    ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
    ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
    if (CRC8 != ahrs_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    baseBringup::checkSN(TYPE_AHRS);
  }
  else if (head_type == TYPE_INSGPS)
  {
    insgps_frame_.frame.header.header_start   = 0xfc;
    insgps_frame_.frame.header.data_type      = TYPE_INSGPS;
    insgps_frame_.frame.header.data_size      = check_len[0];
    insgps_frame_.frame.header.serial_num     = check_sn[0];
    insgps_frame_.frame.header.header_crc8    = head_crc8[0];
    insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
    if (CRC8 != insgps_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    else if(debug_log_)
    {
      std::cout << "header_crc8 matched." << std::endl;
    }
    
    baseBringup::checkSN(TYPE_INSGPS);
  }
  if (head_type == TYPE_IMU)
  {
    uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(imu).");
      return;
    }
    else if(imu_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
    
  }
  else if (head_type == TYPE_AHRS)
  {
    uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(ahrs).");
      return;
    }
    else if(ahrs_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
  }
  else if (head_type == TYPE_INSGPS)
  {
    uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
    size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(insgps).");
      return;
    }
    else if(insgps_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
    
  }

  // publish magyaw topic
  if (head_type == TYPE_AHRS)
  {
    // publish imu topic
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = imu_frame_id_.c_str();
    Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                              ahrs_frame_.frame.data.data_pack.Qx,
                              ahrs_frame_.frame.data.data_pack.Qy,
                              ahrs_frame_.frame.data.data_pack.Qz);
    Eigen::Quaterniond q_r =                          
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr =                          
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_xiao_rr =
        Eigen::AngleAxisd( 3.14159/2, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
    imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
    imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
    imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;

    imu_pub_.publish(imu_data);

    Eigen::Quaterniond rpy_q(imu_data.orientation.w,
                              imu_data.orientation.x,
                              imu_data.orientation.y,
                              imu_data.orientation.z);
    geometry_msgs::Pose2D pose_2d;
    double magx, magy, magz, roll, pitch;
    magx  = -imu_frame_.frame.data.data_pack.magnetometer_x;
    magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
    magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
    Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
    roll  = EulerAngle[2];
    pitch = EulerAngle[1];

    double magyaw;
    magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
    pose_2d.theta = magyaw;
    mag_pose_pub_.publish(pose_2d);
  }
}

void baseBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
  // return magyaw;
}

void baseBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_3.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_4.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_5.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_6.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_7.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_8.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

void baseBringup::setWriteCS(int len)
{
	uint8_t ck = 0x00;
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_write_.write_tmp[i];
	}
	pack_write_.write_tmp[len - 1] = ck;
  lock.unlock();
}

void baseBringup::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  if(debug_log_){
    std::cout << "joyCallback" << std::endl;
  }
  //mode_switch 
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  if (msg->buttons[0]==1)//turn off joy mode
  {
    controll_type_ = MOTOR_MODE_CMD;
    std::cout << "controll_type_ turn to MOTOR_MODE_CMD:" << controll_type_ << std::endl;

  }
  else if (msg->buttons[1] == 1)//turn on joy mode
  {
    controll_type_ = MOTOR_MODE_JOY;
    std::cout << "controll_type_ turn to MOTOR_MODE_JOY:" << controll_type_ << std::endl;
  }
  
  //set speed 
  if (msg->axes[6] == 1)        //twist_speed down
  {
    if(debug_log_){
      std::cout << "twist_speed down" << std::endl;
    }
    twist_gain_ -= 0.1;

  }else if(msg->axes[6] == -1)  //twist_speed up
  {
    if(debug_log_){
      std::cout << "twist_speed up" << std::endl;
    }
    twist_gain_ += 0.1;
  }
  if (msg->axes[7] == -1)      //linear_speed down
  {
    if(debug_log_){
      std::cout << "linear_speed down" << std::endl;
    }
    linear_gain_ -= 0.1;
  }else if(msg->axes[7] == 1)  //linear_speed up
  {
    if(debug_log_){
      std::cout << "linear_speed up" << std::endl;
    }
    linear_gain_ += 0.1;
  }
  //write speed
  double linear_x  = linear_gain_ * msg->axes[1];  // linear_gain_ = 0.3
  double linear_y  = linear_gain_ * msg->axes[0];  // twist_gain_  = 0.2  
  double angular_z = twist_gain_  * msg->axes[2];  // msg->axes[]  = [-1,1]

  if (debug_log_){
    cout << "linear_x=" << linear_x << "linear_y=" << linear_y << "angular_z=" << angular_z << endl;
  }  
  joy_linear_x_  =  linear_x;
  joy_linear_y_  =  linear_y;
  joy_angular_z_ =  angular_z;
  lock.unlock();
  return;
}

void baseBringup::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  if (controll_type_ != MOTOR_MODE_CMD)
  {
    lock.unlock();
    return;
  }
  if(debug_log_){
    std::cout << "vel mode" << std::endl;
  }
  cmd_linear_x_  =  msg -> linear.x;
  cmd_linear_y_  =  msg -> linear.y;
  cmd_angular_z_ =  msg -> angular.z;
  last_cmd_time_ = ros::Time::now();
  lock.unlock();
  return;
}

bool baseBringup::checkCS(int len){
  uint8_t ck = 0;
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_read_.read_tmp[i];
	}
  return pack_read_.read_tmp[len - 1] == ck;
  // return true;
}

void baseBringup::processBattery()
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  current_battery_percent_ = (int)pack_read_.pack.battery_percent;
  lock.unlock();
  sensor_msgs::BatteryState battery_msg;
  battery_msg.power_supply_status     = 2; // 放电中 即正在运行
  battery_msg.power_supply_health     = 1; // 良好
  battery_msg.power_supply_technology = 0; // UNKNOWN
  battery_msg.present    = true;           // 存在电池
  battery_msg.percentage = current_battery_percent_; // 电池电量百分比
  battery_pub_.publish(battery_msg);
}

void baseBringup::processOdometry(){
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  last_time_ = current_time_;
  lock.unlock();
  double vw1,vw2,vw3,vw4;
  vw1 = -pack_read_.pack.data.pluse_w1 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw2 =  pack_read_.pack.data.pluse_w2 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw4 = -pack_read_.pack.data.pluse_w3 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw3 =  pack_read_.pack.data.pluse_w4 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);

  double Vx,Vy,Vth;
  Vx  = ( vw1+vw2+vw3+vw4)/4;
  Vy  = 0.975*(-vw1+vw2-vw3+vw4)/4;
  Vth = (-vw1+vw2+vw3-vw4)/(4*(base_shape_a_+base_shape_b_));

  double delta_x = (Vx * cos(th_) - Vy * sin(th_)) * dt;
  double delta_y = (Vx * sin(th_) + Vy * cos(th_)) * dt;
  double delta_th = Vth * dt;
  lock.lock();
  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;
  lock.unlock();
  nav_msgs::Odometry odom_tmp;
  odom_tmp.header.stamp = ros::Time::now();
  odom_tmp.header.frame_id = odom_frame_.c_str();
  odom_tmp.child_frame_id  = base_frame_.c_str();
  odom_tmp.pose.pose.position.x = x_;
  odom_tmp.pose.pose.position.y = y_;
  odom_tmp.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
  odom_tmp.pose.pose.orientation = odom_quat;
  if (!Vx||!Vy||!Vth){
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE;
  }else{
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE2;
  }
  odom_tmp.twist.twist.linear.x  = Vx;
  odom_tmp.twist.twist.linear.y  = Vy;
  odom_tmp.twist.twist.linear.z  = 0.0;
  odom_tmp.twist.twist.angular.x = 0.0;
  odom_tmp.twist.twist.angular.y = 0.0;
  odom_tmp.twist.twist.angular.z = Vth;
  if (!Vx||!Vy||!Vth){
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE;
  }else{
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE2;
  }
  odom_pub_.publish(odom_tmp);
  
  lock.lock();
  current_odom_ = odom_tmp;
  lock.unlock();

  if (ros::param::has("publish_odom_tf"))
  {
    ros::param::get("publish_odom_tf", provide_odom_tf_);
  }
  
  if(provide_odom_tf_)
  {
    geometry_msgs::TransformStamped odom_trans;     /* first, we'll publish the transform over tf */
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_.c_str();
    odom_trans.child_frame_id  = base_frame_.c_str();
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);    /* send the transform */
  }
  updateMileage(odom_tmp.twist.twist.linear.x,odom_tmp.twist.twist.linear.y,dt);
}

bool baseBringup::getMileage(){
  std::fstream fin;
  std::fstream fin_b;
  std::string str_in;
  std::string str_in_b;
  std::stringstream ss;
  ros::NodeHandle pravite_nh("~");
  
	fin.open(Mileage_file_name_.c_str()); //Mileage_backup_file_name_
  fin_b.open(Mileage_backup_file_name_.c_str());
  if (fin.fail() && fin_b.fail())
  {
    ROS_ERROR("open Mileage files error, will creat a new file! \n");
    Mileage_sum_ = 0.0;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
    return false;
  }
  if (!fin.fail()){
    fin >> str_in;
    fin.close();
  }
  if (!fin_b.fail()){
      fin_b >> str_in_b;
      fin_b.close();
  }
  if (str_in != "")
  {
    ss << str_in;
    ss >> Mileage_sum_;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
    ss.clear();
  }
  else if (str_in_b != "")
  {
    ss << str_in_b;
    ss >> Mileage_sum_;
    ss.clear();
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  }
  else
  {
    ROS_ERROR("Mileage_files empty. \n");
    Mileage_sum_ = 0.0;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  }
  return true;
}

bool baseBringup::updateMileage(double vx, double vy, double dt){
  double speed  = sqrt(vx*vx + vy*vy);
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  Mileage_sum_ += speed * dt;
  lock.unlock();
  double d_Mileage = abs(Mileage_sum_ - Mileage_last_);
  if (d_Mileage < 0.1)
  {
    return true;
  }
  FILE* fout = std::fopen(Mileage_file_name_.c_str(), "w");
  if (fout)
	{
		std::fprintf(fout,"%lf\n",Mileage_sum_);
		std::fclose(fout);
	}
  FILE* fout_b = std::fopen(Mileage_backup_file_name_.c_str(), "w");
  if (fout_b)
	{
		std::fprintf(fout_b,"%lf\n",Mileage_sum_);
		std::fclose(fout_b);
	}
  ros::NodeHandle pravite_nh("~");
  pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  Mileage_last_ = Mileage_sum_;
  return true;
}

void baseBringup::callHandle()
{
  serial_.open();
}

}//namespace ucarController

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_bringup");
  ucarController::baseBringup bp;

  return 0;
}