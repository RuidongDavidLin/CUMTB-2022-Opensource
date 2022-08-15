#ifndef DATA_STRUCT_H_
#define DATA_STRUCT_H_

#include <iostream>
namespace ucarController{
//for read ===============
#pragma pack(1)
struct wheel_pluse_struct{
  int16_t pluse_w1;
  int16_t pluse_w2;
  int16_t pluse_w3;
  int16_t pluse_w4;
};
union data_read{
  struct wheel_pluse_struct data;
  uint8_t wheel_tmp[8];
};
#pragma pack()

#pragma pack(1)
struct read_pack_struct{
  uint16_t head;                    //2 byte
  uint8_t  ver;                     //1 byte
  uint8_t  len;                     //1 byte
  struct   wheel_pluse_struct data; //8 byte
  uint8_t  battery_percent;         //1 byte(new)
  uint8_t  cs_code;                 //1 byte
};                           //total 14 byte
struct read_tmp_struct{
  uint8_t head[2];
  uint8_t read_msg[12];      // 11 + 1(new)
};                           //total 14 byte

union pack_read{
  struct read_pack_struct pack;
  read_tmp_struct read_msg;
  uint8_t read_tmp[14];      //13 + 1(new)
};
#pragma pack()
//for read ----------------------

#pragma pack(1)
struct write_data_struct{
  int16_t vel_x;
  int16_t vel_y;
  int16_t vel_angle_z;
};
union data_write{
  struct write_data_struct data;
  uint8_t wrtie_tmp[6];
};
#pragma pack()

#pragma pack(1)
struct write_pack_struct{
  uint16_t head;                    //2 byte
  uint8_t  ver;                     //1 byte
  uint8_t  len;                     //1 byte
  struct   wheel_pluse_struct data; //8 byte
  uint8_t  green_value;              //1 byte(new)
  uint8_t  red_value;            //1 byte(new)
  uint8_t  blue_value;             //1 byte(new)
  uint8_t  cs_code;                 //1 byte
};                           //total 13 + 3(new) byte

union pack_write{
  struct write_pack_struct pack;
  uint8_t write_tmp[16]; // 13 + 3(new)
};
#pragma pack()

}//namespace AIcarController
#endif//DATA_STRUCT_H_
