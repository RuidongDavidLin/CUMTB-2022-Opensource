#ifndef __PROTOCOL_PROC_UNIT_H
#define __PROTOCOL_PROC_UNIT_H

#include <pthread.h>
#include "queue_simple.h"


#ifdef __cplusplus
extern "C" {
#endif
	
#define __PACKED __attribute__ ((packed))
#define BUFFSIZE 4096
#define MAX_PARSE_SIZE (16*1024*12)
#define whether_show_log  0
/*
业务所需的数�?
*/
typedef struct __PACKED
{
	unsigned int handle;
	unsigned char version;
	unsigned char opcode;
	unsigned char modId;
	unsigned char msgId;
	unsigned char *data;
	unsigned int length;		//data的数据长�?
}business_msg_t;

typedef int(*pfunc_business_proc_callback)(business_msg_t businessMsg);	//业务处理
typedef void(*pfunc_err_proc)(void);	//包解析出错处�?
typedef int(*pfunc_send_msg)(const unsigned char *data, int size);	//底层发送消息机�?
typedef int(*pfunc_recv_msg)(unsigned char *data, int size);		//底层接收消息机制

//opcode
typedef enum {
	OPCODE_REQUEST = 0x01,
	OPCODE_RESPONSE,
	OPCODE_INDICATION,			//主设备推送消息给从设备的方法
	OPCODE_NOTIFICATION 		//从设备推送消息给主设备的方法
	// opcode_request	= 0x1,			
	// opcode_response,
	// opcode_indication,			//APP推送消息给设备的方�?
	// opcode_notification			//设备推送消息给APP的方�?
}opcode_type_t;

/******************************************
header format:
	StartFlag	Handle	Version	Opcode	Length
	4BYTE		4BYTE	1BYTE	1BYTE	4BYTE
*******************************************/
typedef struct __PACKED {
	unsigned int start_flag;			//固定�?xFFAA5500，每一个数据包固定的开始序�?
    unsigned int handle;				//usb 通信协议中填�?
    unsigned char version;						//协议版本
    unsigned char opcode;						//opcode_type_t
    unsigned int length;
}message_client_header_t;

/******************************************
protocal format:
	Header	Payload	Checksum
	14BYTE	nBYTE	1BYTE
*******************************************/
typedef struct __PACKED {
	message_client_header_t header;
	unsigned char data[1]; // include checksum
}message_client_t;

//typdef
typedef enum {
    PARSE_HEAD,
    PARSE_HANDLE,
    PARSE_VERSION,
    PARSE_OPCODE,
    PARSE_LENGTH,
    PARSE_PAYLOAD,
    PARSE_CHECKSUM,
    PARSE_SUCCESS
} PARSE_POSITION;

typedef enum
{
    PARSE_NO_ERROR = 0,
    PARSE_DATA_DEFICIENCIES,           //数据不足
    PARSE_START_FLAG_ERROR,				//起始符出�?
    PARSE_CHECKSUM_ERROR,               //校验和出�?
    PARSE_LENGTH_ERROR,                 //数据长度错误
    PARSE_NOT_ENOUGH_MEMORY,            //内存不足
}PARSE_ERROR;

typedef struct {
    message_client_header_t message;
    void *data;
    unsigned char left[MAX_PARSE_SIZE];
    int left_pos;
    int left_size;
    PARSE_POSITION parse_pos;
} parse_package_t;


typedef struct {
    int handle_fd;
    int running;
    pthread_t send_tid;
    pthread_t recv_tid;
    queue_t *send_queue;
    unsigned char read_buf[BUFFSIZE];
    int rb_length;				//read buf的数据长�?
}protocol_service_handle_t;


typedef struct 
{
	pfunc_send_msg sendMsg;
	pfunc_recv_msg recvMsg;
	pfunc_business_proc_callback businessProcCb;
	pfunc_err_proc errProc;
	protocol_service_handle_t protocalServiceHandle;
	parse_package_t parser;
}protocal_service_t;

protocal_service_t audioService;
protocal_service_t audioService1;


//define


//function
int start_protocol_service(protocal_service_t *pService);
void stop_protocol_service(protocal_service_t *pService);
int protocol_proc_init(pfunc_send_msg sendMsg, pfunc_recv_msg recvMsg, pfunc_business_proc_callback businessProcCb, pfunc_err_proc errProc);
// int protocol_proc_release();
// int parse_message(unsigned char *data, int size);

int send_request_message(unsigned char modId, unsigned char msgId, unsigned char* data, int size, protocal_service_t *pService);
int send_response_message(unsigned char modId, unsigned char msgId, unsigned char* data, int size, protocal_service_t *pService);
int send_indication_message(unsigned char modId, unsigned char msgId, unsigned char* data, int size, protocal_service_t *pService);
int send_notification_message(unsigned char modId, unsigned char msgId, unsigned char* data, int size, protocal_service_t *pService);
int send_message(unsigned char *data, int size, protocal_service_t *pService);
// int send_message(unsigned char modId, unsigned char msgId, unsigned char* data, int size, opcode_type_t opcode);


// unsigned char crc8_atm(unsigned char *ptr, unsigned char len);

#ifdef __cplusplus
}
#endif

#endif
