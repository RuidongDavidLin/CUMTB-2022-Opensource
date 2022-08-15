#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include "hidapi.h"
#include "string.h"
#define SAMPLE_RATE_16K (16000)
#define SAMPLE_RATE_8K (8000)
#define MAX_GRAMMARID_LEN (32)
#define MAX_PARAMS_LEN (1024)
#define FRAME_LEN 640
#define BUFFER_SIZE 4096

static char *whole_result =(char*)" "; //识别到的所有内容

//更新离线识别语法的contact槽（语法文件为此示例中使用的call.bnf）
typedef struct _UserData
{
	int build_fini;						//标识语法构建是否完成
	int update_fini;					//标识更新词典是否完成
	int errcode;						//记录语法构建或更新词典回调错误码
	char grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
} UserData;

typedef struct _Recognise_Result
{
	bool whether_recognised; //是否检测成功
	char *whole_content;	 //识别到的所有内容
	char fail_reason[32];	//　检测识别的原因,如果检测成功则为空;
} Recognise_Result;

typedef struct _Effective_Result
{
	char effective_word[32];  //识别到的有效文本
	int effective_confidence; //有效文本的置信度
} Effective_Result;

#ifdef __cplusplus
extern "C"
{
#endif

	int build_grammar(UserData *udata);  //构建离线识别语法网络
	int update_lexicon(UserData *udata); //更新离线识别语法词典

	int build_grm_cb(int ecode, const char *info, void *udata);
	int update_lex_cb(int ecode, const char *info, void *udata);
	//static void show_result(char *string, char is_over);
	void on_result(const char *result, char is_last);
	void on_speech_begin();
	void on_speech_end(int reason);
	static void demo_file(const char *audio_file, const char *session_begin_params);
	static void demo_no_pcm_file(const char *p_pcm, const char *session_begin_params);
	static void demo_mic(const char *session_begin_params);

	//Recognise_Result deal_with(unsigned char *path, char *source_path,char *lex_na);
    Recognise_Result deal_with(unsigned char *path, char *jet_path, char *grammer_path, char *bnf_path, char *lex_na);
	int run_asr(UserData *udata, unsigned char *path);

#ifdef __cplusplus
}
#endif
