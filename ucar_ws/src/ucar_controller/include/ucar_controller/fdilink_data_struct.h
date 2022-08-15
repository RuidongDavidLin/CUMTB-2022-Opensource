#ifndef FDILINK_DATA_STRUCT_H_
#define FDILINK_DATA_STRUCT_H_

#include <iostream>

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x54 //84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

namespace FDILink{
#pragma pack(1)
struct fdilink_header
{
	uint8_t  header_start;
	uint8_t  data_type;
	uint8_t  data_size;
	uint8_t  serial_num;
	uint8_t  header_crc8;
    uint8_t  header_crc16_h;
	uint8_t  header_crc16_l;
};
#pragma pack()

#pragma pack(1)
struct IMUData_Packet_t
{
	float gyroscope_x;          //unit: rad/s
	float gyroscope_y;          //unit: rad/s
	float gyroscope_z;          //unit: rad/s
	float accelerometer_x;      //m/s^2
	float accelerometer_y;      //m/s^2
	float accelerometer_z;      //m/s^2
	float magnetometer_x;       //mG
	float magnetometer_y;       //mG
	float magnetometer_z;       //mG
	float imu_temperature;      //C
	float Pressure;             //Pa
	float pressure_temperature; //C
	int64_t Timestamp;          //us
};
#pragma pack()

struct AHRSData_Packet_t
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	int64_t Timestamp; //unit: us
};
#pragma pack(4)
struct INSGPSData_Packet_t
{
	float BodyVelocity_X;       //4
	float BodyVelocity_Y;       //4
	float BodyVelocity_Z;       //4
	float BodyAcceleration_X;   //4
	float BodyAcceleration_Y;   //4
	float BodyAcceleration_Z;   //4
	double Location_North;      //8
	double Location_East;       //8
	double Location_Down;
	float Velocity_North;
	float Velocity_East;
	float Velocity_Down;
	float Acceleration_North;
	float Acceleration_East;
	float Acceleration_Down;
	float Pressure_Altitude;
	int64_t Timestamp;
};
#pragma pack()

//for IMU=========================
#pragma pack(1)
struct read_imu_struct{
  fdilink_header     header;    //7                
  union data
  {
	IMUData_Packet_t   data_pack; //56
	uint8_t            data_buff[56]; //56
  }data;
  uint8_t            frame_end; //1                  
};            

struct read_imu_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[57];
};                           

union imu_frame_read{
  struct read_imu_struct frame;
  read_imu_tmp read_buf;
  uint8_t read_tmp[64];
};
#pragma pack()
//for IMU------------------------

//for AHRS=========================
#pragma pack(1)
struct read_ahrs_struct{
  fdilink_header     header;    //7                
  union data
  {
	AHRSData_Packet_t  data_pack; //48
	uint8_t            data_buff[48]; //48
  }data;
  uint8_t            frame_end; //1                  
};       

struct read_ahrs_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[49];
};                           

union ahrs_frame_read{
  struct read_ahrs_struct frame;
  read_ahrs_tmp read_buf;
  uint8_t read_tmp[56];
};
#pragma pack()
//for AHRS------------------------

//for INSGPS=========================
#pragma pack(1)
struct read_insgps_struct{
  fdilink_header     header;    //7                
  union data
  {
	INSGPSData_Packet_t  data_pack; //48
	uint8_t              data_buff[84]; //48
  }data;
  uint8_t            frame_end; //1                  
};   
                          
struct read_insgps_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[85];
};                          

union insgps_frame_read{
  struct read_insgps_struct frame;
  read_insgps_tmp read_buf;
  uint8_t read_tmp[92];
};
#pragma pack()
//for INSGPS------------------------


}//namespace FDILink
#endif//FDILINK_DATA_STRUCT_H_