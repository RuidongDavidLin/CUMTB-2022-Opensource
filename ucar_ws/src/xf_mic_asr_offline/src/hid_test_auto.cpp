/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
#include "user_interface.h"
#include <string>
#include <locale>
#include <codecvt>
#include <ctime>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_mic_asr_offline/Get_Offline_Result_srv.h>
#include <xf_mic_asr_offline/Set_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Set_Led_On_srv.h>
#include <xf_mic_asr_offline/Get_Major_Mic_srv.h>
#include <xf_mic_asr_offline/Pcm_Msg.h>
#include <xf_mic_asr_offline/Start_Record_srv.h>
#include <xf_mic_asr_offline/Set_Awake_Word_srv.h>
#include <xf_mic_asr_offline/Get_Awake_Angle_srv.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>

ros::Publisher pub_pcm;
ros::Publisher pub_awake_angle;
ros::Publisher pub_start_lable;
ros::Subscriber sub_record_start;
ros::Subscriber sub_targrt_led_on;
ros::Subscriber sub_get_major_mic;

ros::Publisher major_mic_pub;
ros::Publisher recognise_result_pub;
std::string awake_angle_topic = "/mic/awake/angle";
std::string pcm_topic = "/mic/pcm/deno";
std::string major_mic_topic = "/mic/major_mic";
std::string lable_topic = "/start_lable";

int offline_recognise_switch = 0; //离线识别默认开关
std::vector<char> pcm_buf;		  //音频流缓冲区

bool Get_request_mic_id = false;
bool Set_request_mic_id = false;
bool Set_request_led_id = false;
bool Set_request_awake_word = false;
bool Get_request_awake_angle = false;
using namespace std;

/*获取文件大小*/
int FileSize(const char *fname)
{
	struct stat statbuf;
	if (stat(fname, &statbuf) == 0)
		return statbuf.st_size;
	return -1;
}

std::wstring s2ws(const std::string &str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

std::string ws2s(const std::wstring &wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

//判断是否是整数
int isnumber(char *a, int count_need)
{
	int len = strlen(a);
	if (len > count_need)
	{
		return -1;
	}
	int j = 0;
	for (int i = 0; i < len; i++)
	{
		if (a[i] <= 57 && a[i] >= 48)
		{
			j++;
		}
	}
	if (j == len)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

//字符串与字符数组拼接
char *join(std::string b, char *s2)
{
	char s1[600] = "";
	try
	{
		strcpy(s1, b.c_str());
	}
	catch (...)
	{
		cout << ">>>>>join拷贝失败" << endl;
	}
	char *result = (char *)malloc(strlen(s1) + strlen(s2) + 1);
	if (result == NULL)
		exit(1);

	try
	{
		strcpy(result, s1);
		strcat(result, s2);
	}
	catch (...)
	{
		cout << ">>>>>join拷贝失败" << endl;
	}
	return result;
}

//麦克风通信回调函数
int business_proc_callback(business_msg_t businessMsg)
{
	int res = 0;
	char *fileName = join(source_path, DENOISE_SOUND_PATH);
	char *fileName_ori = join(source_path, ORIGINAL_SOUND_PATH);
	static int index = 0;
	unsigned char buf[4096];
	//printf("business proc modId = %d, msgId = %d, size = %d", businessMsg.modId, businessMsg.msgId, businessMsg.length);
	switch (businessMsg.modId)
	{
	case 0x01:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//printf(">>>>>您已开启录音\n");
			}
		}
		else if (businessMsg.msgId == 0x02)
		{
			/*获取音频流，并保存在本地*/
			if (save_pcm_local)
			{
				char *denoise_sound_path = join(source_path, DENOISE_SOUND_PATH);
				if (-1 != FileSize(denoise_sound_path))
				{
					int file_size = FileSize(denoise_sound_path);
					if (file_size > max_pcm_size) //超出最大文件限制,将删除,以节省磁盘空间
					{
						remove(denoise_sound_path);
					}
				}
				get_denoised_sound(denoise_sound_path, businessMsg.data);
			}
			int len = PCM_MSG_LEN;
			char *pcm_buffer = new char[len]; //在堆中创建空间
#if whether_print_log
			if (pcm_buffer == NULL)
			{
				cout << "buffer is null" << endl;
			}
			else
			{
				cout << "buffer alloced successfully" << endl;
			}
#endif
			try
			{
				memcpy(pcm_buffer, businessMsg.data, len);
			}
			catch (...)
			{
				cout << ">>>>>拷贝失败" << endl;
			}

			try
			{
				for (int i = 0; i < len; i++)
				{
					pcm_buf.push_back(pcm_buffer[i]);
				}
			}
			catch (...)
			{
				cout << ">>>>>赋值失败" << endl;
			}
#if whether_print_log
			printf("pcm_buf.size:%d", pcm_buf.size());
#endif
			if (pcm_buf.size() == PCM_MSG_LEN) //满足指定帧率要求的字节数才发送
			{
				xf_mic_asr_offline::Pcm_Msg pcm_data;
				vector<char>::iterator it;
				for (it = pcm_buf.begin(); it != pcm_buf.end(); it++)
				{
					pcm_data.pcm_buf.push_back(*it);
				}
				pcm_data.length = len;
				pub_pcm.publish(pcm_data);
				pcm_buf.clear();
			}
			delete[] pcm_buffer; //释放创建的堆空间
		}
		else if (businessMsg.msgId == 0x03)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//发布关闭前剩余的音频流
				xf_mic_asr_offline::Pcm_Msg pcm_data;
				vector<char>::iterator it;
				for (it = pcm_buf.begin(); it != pcm_buf.end(); it++)
				{
					pcm_data.pcm_buf.push_back(*it);
				}
				pcm_data.length = pcm_buf.size();
				pub_pcm.publish(pcm_data);
				pcm_buf.clear();
				//printf(">>>>>您已停止录音\n");
			}
		}
		else if (businessMsg.msgId == 0x04)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				printf(">>>>>开/关原始音频成功\n");
			}
		}
		else if (businessMsg.msgId == 0x05)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//printf(">>>>>设置主麦克风成功\n");
			}
		}
		else if (businessMsg.msgId == 0x06)
		{
			get_original_sound(fileName_ori, businessMsg.data);
		}
		else if (businessMsg.msgId == 0x07)
		{
			unsigned char key2[] = "beam";
			try
			{
				int major_id = whether_set_succeed(businessMsg.data, key2);
				major_mic_id = major_id;
				Get_request_mic_id = true;
				printf(">>>>>主麦克风id为%d号麦克风\n", major_mic_id);
			}
			catch (...)
			{
				Get_request_mic_id = false;
			}
		}
		else if (businessMsg.msgId == 0x08)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				Set_request_mic_id = true;
				printf("\n>>>>>设置主麦克风成功\n");
			}
			else
			{
				Set_request_mic_id = false;
			}
		}
		else if (businessMsg.msgId == 0x09)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				Set_request_led_id = true;
				printf("\n>>>>>设置灯光成功\n");
			}
			else
			{
				Set_request_led_id = false;
			}
		}
		break;
	case 0x02:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key1[] = "beam";
			unsigned char key2[] = "angle";
			major_mic_id = get_awake_mic_id(businessMsg.data, key1);
			mic_angle = get_awake_mic_angle(businessMsg.data, key2);
			if (major_mic_id <= 5 && major_mic_id >= 0 && mic_angle <= 360 && mic_angle >= 0)
			{
				if_awake = 1;
				led_id = get_led_based_angle(mic_angle);
				int ret1 = set_major_mic_id(major_mic_id);
				int ret2 = set_target_led_on(led_id);
				if (ret1 == 0 && ret2 == 0)
				{
					printf(">>>>>第%d个麦克风被唤醒\n", major_mic_id);
					printf(">>>>>唤醒角度为:%d\n", mic_angle);
					printf(">>>>>已点亮%d灯\n", led_id);
					Get_request_awake_angle = true;
					std_msgs::Int32 awake_angle;
					awake_angle.data = mic_angle;
					pub_awake_angle.publish(awake_angle);
					std_msgs::Bool ret;
					ret.data= true;
					pub_start_lable.publish(ret);
				}
			}
		}
		else if (businessMsg.msgId == 0x08)
		{
			unsigned char key1[] = "errstring";
			int result = whether_set_awake_word(businessMsg.data, key1);
			if (result==0)
			{
				Set_request_awake_word = true;
				printf("\n>>>>>唤醒词设置成功\n");
			}
			else if (result==-2)
			{
				Set_request_awake_word = false;
				printf("\n>>>>>唤醒词设置失败\n");
			}
		}

		break;
	case 0x03:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key[] = "status";
			int status = whether_set_succeed(businessMsg.data, key);
			char protocol_version[40]; 
			int ret = get_protocol_version(businessMsg.data,protocol_version);
			printf(">>>>>麦克风%s,软件版本为:%s,协议版本为:%s\n", (status == 0 ? "正常工作" : "正在启动"),get_software_version(),protocol_version);
			if (status == 1)
			{
				char *fileName = join(source_path,SYSTEM_CONFIG_PATH);
				send_resource_info(fileName, 0);
			}
			else
			{
				is_boot = 1;
			}
		}

		break;

	case 0x04:
		if (businessMsg.msgId == 0x01)
		{
			whether_set_resource_info(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x03) //文件接收结果
		{
			whether_set_resource_info(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x04) //查看设备升级结果
		{
			whether_upgrade_succeed(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x05) //下发文件
		{
			char *fileName_system_path = join(source_path, SYSTEM_PATH);
			;
			send_resource(businessMsg.data, fileName_system_path, 1);
		}
		else if (businessMsg.msgId == 0x08) //获取升级配置文件
		{
			printf("config.json: %s", businessMsg.data);
		}
		break;

	default:
		break;
	}
	return 0;
}
/*用于显示离线命令词识别结果*/
Effective_Result show_result(char *string) //
{
	Effective_Result current;
	if (strlen(string) > 250)
	{
		char asr_result[32];	//识别到的关键字的结果
		char asr_confidence[3]; //识别到的关键字的置信度
		char *p1 = strstr(string, "<rawtext>");
		char *p2 = strstr(string, "</rawtext>");
		int n1 = p1 - string + 1;
		int n2 = p2 - string + 1;

		char *p3 = strstr(string, "<confidence>");
		char *p4 = strstr(string, "</confidence>");
		int n3 = p3 - string + 1;
		int n4 = p4 - string + 1;
		for (int i = 0; i < 32; i++)
		{
			asr_result[i] = '\0';
		}

		strncpy(asr_confidence, string + n3 + strlen("<confidence>") - 1, n4 - n3 - strlen("<confidence>"));
		asr_confidence[n4 - n3 - strlen("<confidence>")] = '\0';
		int confidence_int = 0;
		confidence_int = atoi(asr_confidence);
		if (confidence_int >= confidence)
		{
			strncpy(asr_result, string + n1 + strlen("<rawtext>") - 1, n2 - n1 - strlen("<rawtext>"));
			asr_result[n2 - n1 - strlen("<rawtext>")] = '\0'; //加上字符串结束符。
		}
		else
		{
			strncpy(asr_result, "", 0);
		}

		current.effective_confidence = confidence_int;
		strcpy(current.effective_word, asr_result);
		return current;
	}
	else
	{
		current.effective_confidence = 0;
		strcpy(current.effective_word, " ");
		return current;
	}
}

/*获取离线命令词识别结果*/
bool Get_Offline_Recognise_Result(xf_mic_asr_offline::Get_Offline_Result_srv::Request &req,
								  xf_mic_asr_offline::Get_Offline_Result_srv::Response &res)
{
	offline_recognise_switch = req.offline_recognise_start;
	if (offline_recognise_switch == 1) //如果是离线识别模式
	{
		ROS_INFO("open the offline recognise mode ...\n");
		const char *file = join(source_path, DENOISE_SOUND_PATH);
		remove(file);
		start_to_record_denoised_sound();
		int time_per = time_per_order;
		if (req.time_per_order > 0)
		{
			time_per = req.time_per_order;
		}
		while (time_per--)
		{
			sleep(1);
		}
		finish_to_record_denoised_sound();
		char *pcm_path = join(source_path, DENOISE_SOUND_PATH);
		std::string transfom_pcm = pcm_path;
		unsigned char *unsignend_pcm_path = (unsigned char *)transfom_pcm.c_str();
		std::string begin = "fo|";
		char *jet_path = join((begin + source_path), ASR_RES_PATH);
		char *grammer_build_path = join(source_path, GRM_BUILD_PATH);
		char *bnf_path = join(source_path, GRM_FILE);
		
		Recognise_Result result = deal_with(unsignend_pcm_path, jet_path, grammer_build_path, bnf_path, LEX_NAME);
		if (result.whether_recognised)
		{
			if (result.whole_content) //确保有反馈
			{
				/**为了在终端可视化方便,可将本行注释，若需查看完整反馈，可解除屏蔽**/
				printf(">>>>>全部返回结果:　[ %s ]\n", result.whole_content);
				Effective_Result effective_ans = show_result(result.whole_content); //
				if (effective_ans.effective_confidence >= req.confidence_threshold) //如果大于置信度阈值则进行显示或者其他控制操作
				{
					printf(">>>>>是否识别成功:　[ %s ]\n", "是");
					printf(">>>>>关键字的置信度: [ %d ]\n", effective_ans.effective_confidence);
					printf(">>>>>关键字识别结果: [ %s ]\n", effective_ans.effective_word);
					/*发布结果*/
					res.result = "ok";
					res.fail_reason = "";
					std::wstring wtxt = s2ws(effective_ans.effective_word);
					std::string txt_uft8 = ws2s(wtxt);
					res.text = txt_uft8;
				}
				else
				{
					printf(">>>>>是否识别成功:　[ %s ]\n", "否");
					printf(">>>>>关键字的置信度: [ %d ]\n", effective_ans.effective_confidence);
					res.result = "fail";
					res.fail_reason = "low_confidence error or 11212_license_expired_error";
					res.text = " ";
					printf(">>>>>关键字置信度较低，文本不予显示\n");
				}
			}
			else
			{
				res.result = "fail";
				res.fail_reason = "no_valid_sound error";
				res.text = " ";
				printf(">>>>>未能检测到有效声音,请重试\n");
			}
		}
		else
		{
			res.result = "fail";
			res.fail_reason = result.fail_reason;
			res.text = " ";
			printf(">>>>>是否识别成功:　[ %s ]\n", "否");
			printf(">>>>>错误原因: [ %s ]\n", result.fail_reason);
		}
		sleep(1);
		ROS_INFO("send result to client,stop the offline_recognise mode\n");
	}
	ROS_INFO("close the offline recognise mode ...\n");
	return true;
}

/*
content:获取麦克风音频,if msg==1,开启录音并实时发布，若msg==0,关闭录音
data :20200407 PM
*/
bool Record_Start(xf_mic_asr_offline::Start_Record_srv::Request &req, xf_mic_asr_offline::Start_Record_srv::Response &res)
{
	if (req.whether_start == 1)
	{
		ROS_INFO("got topic request,start to record ...\n");
		int ret1 = start_to_record_denoised_sound();
		if (ret1 == 0)
		{
			res.result = "ok";
			res.fail_reason = "";
		}
		else
		{
			res.result = "fail";
			res.fail_reason = "mic_did_not_open_error";
		}
	}
	else if (req.whether_start == 0)
	{
		ROS_INFO("got topic request,stop to record ...\n");
		int ret2 = finish_to_record_denoised_sound();
		if (ret2 == 0)
		{
			res.result = "ok";
			res.fail_reason = "";
		}
		else
		{
			res.result = "fail";
			res.fail_reason = "mic_did_not_open_error";
		}
	}
	return true;
}

/*
content:设置麦克风唤醒词4-6汉字
data :20200407 PM
*/
bool Set_Awake_Word(xf_mic_asr_offline::Set_Awake_Word_srv::Request &req,
					xf_mic_asr_offline::Set_Awake_Word_srv::Response &res)
{
	ROS_INFO("got request,start to correct awake word ...\n");
	if (strlen(req.awake_word.c_str()) >= 12 && strlen(req.awake_word.c_str()) <= 18) //4-6个汉字
	{
		Set_request_awake_word = false;
		int ret = set_awake_word(const_cast<char *>(req.awake_word.c_str()));
		if (ret == -3)
		{
			res.result = "fail";
			res.fail_reason = "mic_did_not_open_error";
			return true;
		}
		clock_t startTime, endTime;
		startTime = clock(); //计时开始
		while (!Set_request_awake_word)
		{
			endTime = clock();											  //计时开始
			if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) //等待时间大于5秒
			{
				res.result = "fail";
				res.fail_reason = "timeout_error";
				Set_request_awake_word = false;
				return true;
			}
		}
		Set_request_awake_word = false;
		res.result = "ok";
		res.fail_reason = "";
	}
	else
	{
		ROS_INFO("got request,stop to correct awake word...\n");
		res.result = "fail";
		res.fail_reason = "invalid_awake word";
	}
	return true;
}

/*
content:设置灯亮,输入参数为0-11.99表示灯光关闭
data :20200407 PM
*/
bool Set_Led_On(xf_mic_asr_offline::Set_Led_On_srv::Request &req,
				xf_mic_asr_offline::Set_Led_On_srv::Response &res)
{
	ROS_INFO("got topic request,start to make the target led on ...\n");
	char str[256] = {0};
	sprintf(str, "%d", req.led_id);
	int ret1 = isnumber(str, 2);
	if (ret1 == 0)
	{
		if (req.led_id >= 0 && req.led_id <= 11 || req.led_id == 99)
		{
			int ret2 = set_target_led_on(req.led_id);
			if (ret2 == 0)
			{
				clock_t startTime, endTime;
				startTime = clock(); //计时开始
				while (!Set_request_led_id)
				{
					endTime = clock();											  //计时开始
					if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) //等待时间大于5秒
					{
						res.result = "fail";
						res.fail_reason = "timeout_error";
						Set_request_led_id = false;
						return true;
					}
				}
				Set_request_led_id = false;
				res.result = "ok";
				res.fail_reason = "";
			}
			else if (ret1 == -3)
			{
				res.result = "fail";
				res.fail_reason = "mic_did_not_open_error";
			}
		}
		else
		{
			res.result = "fail";
			res.fail_reason = "incorrect_led_id_error";
		}
	}
	else
	{
		res.result = "fail";
		res.fail_reason = "incorrect_led_id_error";
	}
	return true;
}

/*
content:设置主麦克风,麦克风共6个,输入参数为0-5.
data :20200407 PM
*/
bool Set_Major_Mic(xf_mic_asr_offline::Set_Major_Mic_srv::Request &req,
				   xf_mic_asr_offline::Set_Major_Mic_srv::Response &res)
{
	ROS_INFO("got topic request,start to make the target led on ...\n");
	char str[256] = {0};
	sprintf(str, "%d", req.mic_id);
	int ret1 = isnumber(str, 1);
	if (ret1 == 0)
	{
		if (req.mic_id >= 0 && req.mic_id <= 5)
		{
			int led_id = get_led_based_mic_id(req.mic_id);
			if (led_id == -3)
			{
				res.result = "fail";
				res.fail_reason = "mic_did_not_open_error";
			}
			else if (led_id == -2)
			{
				res.result = "fail";
				res.fail_reason = "incorrect_mic_id_error";
			}
			else
			{
				int ret2 = set_major_mic_id(req.mic_id);
				int ret3 = set_target_led_on(led_id);
				if (ret2 != -3 && ret2 != -2 && ret3 != -3 && ret3 != -2)
				{
					clock_t startTime, endTime;
					startTime = clock(); //计时开始
					while (!Set_request_mic_id && !Set_request_led_id)
					{
						endTime = clock();											  //计时开始
						if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) //等待时间大于5秒
						{
							res.result = "fail";
							res.fail_reason = "timeout_error";
							Set_request_mic_id = false;
							return true;
						}
					}
					Set_request_mic_id = false;
					
					res.result = "ok";
					res.fail_reason = "";
				}
				else if(ret2 == -3 || ret3 == -3) 
				{
					res.result = "fail";
					res.fail_reason = "mic_did_not_open_error";
				}
			}
		}
		else
		{
			res.result = "fail";
			res.fail_reason = "incorrect_mic_id_error";
		}
	}
	else
	{
		res.result = "fail";
		res.fail_reason = "incorrect_mic_id_error";
	}
	return true;
}

/*
content:获取主麦克风编号,当请求1时,调用该接口.
*/
bool Get_Major_Mic(xf_mic_asr_offline::Get_Major_Mic_srv::Request &req,
				   xf_mic_asr_offline::Get_Major_Mic_srv::Response &res)
{
	ROS_INFO("got request,start to get the major mic id ...\n");
	Get_request_mic_id = false;
	get_major_mic_id();
	clock_t startTime, endTime;
	startTime = clock(); //计时开始
	while (!Get_request_mic_id)
	{
		endTime = clock();											  //计时开始
		if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) //等待时间大于5秒
		{
			res.result = "fail";
			res.fail_reason = "timeout_error";
			Get_request_mic_id = false;
			return true;
		}
	}
	res.result = "ok";
	res.mic_id = major_mic_id;
	Get_request_mic_id = false;
	return true;
}

/*
content:获取主麦克风编号,当请求1时,调用该接口.
*/
bool Get_Awake_Angle(xf_mic_asr_offline::Get_Awake_Angle_srv::Request &req,
					 xf_mic_asr_offline::Get_Awake_Angle_srv::Response &res)
{
	ROS_INFO("got request,start to get the major awake angle ...\n");
	clock_t startTime, endTime;
	startTime = clock(); //计时开始
	while (!Get_request_awake_angle)
	{
		endTime = clock();											  //计时开始
		if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) //等待时间大于5秒
		{
			res.result = "fail";
			res.fail_reason = "timeout_error";
			Get_request_awake_angle = false;
			return true;
		}
	}
	res.result = "ok";
	res.awake_angle = mic_angle;
	return true;
}

/*程序入口*/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "xf_mic_bringup_node");
	ros::NodeHandle ndHandle("~");
	ndHandle.param("confidence", confidence, 0);//离线命令词识别置信度阈值
	ndHandle.param("seconds_per_order", time_per_order, 3); //单次录制音频的时长
	ndHandle.param("source_path", source_path, std::string("./"));
	ndHandle.param("appid", appid, std::string("5e1593ee"));//appid，需要更换为自己的

	/*　topic 发布实时音频文件*/
	pub_pcm = ndHandle.advertise<xf_mic_asr_offline::Pcm_Msg>(pcm_topic, 1);
	/*　topic 发布唤醒角度*/
	pub_awake_angle = ndHandle.advertise<std_msgs::Int32>(awake_angle_topic,true, 1);
	/*　topic 发布主麦克风*/
	major_mic_pub = ndHandle.advertise<std_msgs::Int8>(major_mic_topic, true, 1);
	/*  topic 发布启动标志*/
	pub_start_lable = ndHandle.advertise<std_msgs::Bool>(lable_topic, true, 1);

	/*srv　接收请求，开启录音或关闭录音*/
	ros::ServiceServer service_record_start = ndHandle.advertiseService("start_record_srv", Record_Start);

	/*srv　接收请求，返回离线命令词识别结果*/
	ros::ServiceServer service_get_wav_list = ndHandle.advertiseService("get_offline_recognise_result_srv", Get_Offline_Recognise_Result);

	/*srv 设置主麦克风*/
	ros::ServiceServer service_set_major_mic = ndHandle.advertiseService("set_major_mic_srv", Set_Major_Mic);

	/*srv 获取主麦克风*/
	ros::ServiceServer service_get_major_mic = ndHandle.advertiseService("get_major_mic_srv", Get_Major_Mic);

	/*srv 设置主麦克风*/
	ros::ServiceServer service_set_led_on = ndHandle.advertiseService("set_target_led_on_srv", Set_Led_On);

	/*srv 修改唤醒词*/
	ros::ServiceServer service_set_awake_word = ndHandle.advertiseService("set_awake_word_srv", Set_Awake_Word);

	/*srv 获取当前唤醒角度*/
	ros::ServiceServer service_get_awake_angle = ndHandle.advertiseService("get_awake_angle_srv", Get_Awake_Angle);

	hid_device *handle = NULL;
	handle = hid_open();//开启麦克风设备

	if (!handle)
	{
		printf(">>>>>无法打开麦克风设备，尝试重新连接进行测试\n");
		return -1;
	}
	printf(">>>>>成功打开麦克风设备\n");
	protocol_proc_init(send_to_usb_device, recv_from_usb_device, business_proc_callback, err_proc);
	get_system_status();//获取麦克风状态，是否正常工作

	sleep(1);
	if (!is_boot)
	{
		printf(">>>>>开机中，请稍等！\n");
	}
	while (!is_boot)
	{
		if (is_reboot)
		{
			break;
		}
	}
	printf(">>>>>开机成功！\n");
	
	APPID = &appid[0];
	ros::AsyncSpinner spinner(3);
	spinner.start();
	if (major_mic_id>5 || major_mic_id<0)
	{
		printf(">>>>>未设置主麦，请唤醒或设置主麦\n");
	}
	while (major_mic_id>5 || major_mic_id<0)
	{
		sleep(1);
	}
	printf(">>>>>设置主麦成功！\n");
	ros::waitForShutdown();

	hid_close();
	return 0;
}
