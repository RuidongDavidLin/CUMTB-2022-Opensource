#! /usr/bin/env python2
# -*- coding: utf-8 -*-
#
# 本模块主要用于实现用户交互功能和记录功能，将输出信息全部保存到 LOG_FILE 文件中，关键信息单独保存到 VITAL_LOG_FILE
# 接口：
#       show()               使用方法类似 python3 print, 详见注释
#       get_bool_ans()       见注释
#       get_str_ans()        见注释

from __future__ import print_function
import time

try:
    LOG_FILE = open('log.txt', 'w')
    VITAL_LOG_FILE = open('vital_log.txt', 'w')
except:
    raise RuntimeError('记录文件创建失败')

def get_time():
    '''
    返回一个时间戳字符串 [09:30:24.628]
    '''
    localtime = time.strftime("%H:%M:%S", time.localtime())
    ms = str(time.time())[10:14]
    return ''.join(['[', localtime, ms, ']'])

# 装饰器，看不懂的话不用管它，仿照样例，直接调用下面三个函数就行
# 在待显示的字符串之前增加绿色 [09:30:24.628] 前缀，即 9点30分24.628秒
# 同时把时间戳保存到 LOG_FILE 里
def log(func, f=None):
    global LOG_FILE
    def wrapper(*args, **kwargs):
        localtime = get_time()
        timestamp = ''.join(['\033[32m', localtime, '\033[0m '])    # 设置颜色为绿色
        print(timestamp, end='')
        print(localtime, file=LOG_FILE, end='')                     # 保存输出到文件
        return func(*args, **kwargs)
    return wrapper

@log
def show(*args):
    '''
    显示一段提示信息，该字符串也会保存到 LOG_FILE 中，
    eg.
        show('Hello World')
    '''
    global LOG_FILE
    print(*args)
    print(*args, file=LOG_FILE)

@log
def get_bool_ans(msg):
    '''
    eg.
    调用 get_bool_ans('Do you love me?')
    
    username@pc:~/$   [09:30:24.628] Do you love me? [Y/n]
    
    若用户作出肯定回答'Y'，返回 True
    参数中的字符串 msg 也会保存到 LOG_FILE 中
    '''
    global LOG_FILE
    question = msg + ' [Y/N] '
    print(question, file=LOG_FILE)
    answer = raw_input(question)
    if answer == 'Y' or answer == 'y':
        return True
    else:
        return False

@log
def get_str_ans(msg):
    '''
    调用 get_bool_ans('Do you love me?')
    终端显示：[09:30:24.628] Do you love me?
    返回用户输入的原始字符串

    参数中的字符串 msg 也会保存到 LOG_FILE 中
    '''
    global LOG_FILE
    print(msg, file=LOG_FILE)
    return raw_input(msg)

def save(*args):
    '''
    保存传入的数据，用法同 print
    '''
    global VITAL_LOG_FILE
    print(*args, file=VITAL_LOG_FILE)

if __name__  == '__main__':
    try:
        # 测试样例，可以尝试执行 './interactive.py' 试一试
        name = get_str_ans("what's your name?\n")
        if get_bool_ans('%s, Do you love me?' %name):
            show("You're the light, you're the night~~~")
        else:
            show('I love the way you lie.')
    except KeyboardInterrupt:
        show('\n操作已取消')
        exit(0)