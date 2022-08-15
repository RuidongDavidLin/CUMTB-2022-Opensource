import cv2  # 安装opencv
from utils import getSocket
from camera.camera import Camera1920
import time
from yolo.detect import yolov4
import os
import numpy
from voice.voice_test import playsound  #用wave库和pyaudio库封装的一个简易的播放函数 

#主函数模块
def main(saveImg=True):

    print('正在加载Yolo模型')
    yolo = yolov4()                     # 加载Yolov4-tiny模型
    camera = Camera1920()               # 摄像头函数实例化
    yolo.getLabels(camera.get())        # 获取当前摄像头图像，并获取识别标签(test)
    print('机器视觉模组载入完成')

    # 启动ROS模组(ucar_nav/main.py)
    print('正在启动 ROS 模块')
    os.system("gnome-terminal -e 'bash -c \"python2 /home/ucar/ucar_ws/src/ucar_nav/scripts/main.py; exec bash\"'")
    sk = getSocket()
    print('检测到 ROS 模块进程链接，Yolo识别模型进入阻塞，等待识别')

    # wait->初始阻塞，在接收到第一次通讯后开始判断  recog->启动任务卡片识别，识别完成后语音播报，再将识别结果传回主进程
    state='wait'                            # 进程状态初始化
    ResultCache = {b"B":"未识别到", b"C":"未识别到", b"D":"未识别到"}     # 定义用于存储房间识别结果的字典
    labelCache = []                         # 定义用于存储图片识别结果的列表
    id = 0                                  # 记录每一张图片的id号码，用于图片保存
    Check_result = True                     # 是否进行误识别纠正
    last_room_state = b"B"                  # 记录上一次识别的房间名
    while True:
        room_state=sk.recv(1024)                 # 等待客户端发出指令
        print(room_state)                        # 打印收到的指令
        if room_state == b'play':                  # 任务结束，进行播报任务
            if Check_result:
                ResultCache = [ResultCache[b"B"], ResultCache[b"C"], ResultCache[b"D"]]
                ResultCache = check(ResultCache)
                print(ResultCache)
            playsound('voice/任务完成')
            time.sleep(0.15)
            playsound('voice/B房间为')
            time.sleep(0.15)
            playsound('voice/' + ResultCache[0])
            time.sleep(0.15)
            playsound('voice/C房间为')
            time.sleep(0.15)
            playsound('voice/' + ResultCache[1])
            time.sleep(0.15)
            playsound('voice/D房间为')
            time.sleep(0.15)
            playsound('voice/' + ResultCache[2])
            break
        sk.send(b"Get")
        state =sk.recv(1024)
        print('test')
        if state == b'recog':
            print('开始采集图像')
            time.sleep(0.2)
            frame = camera.get()
            if last_room_state != room_state:
                labelCache =[]
            labels,image = yolo.getLabelsAndImg(frame)
            id += 1
            cv2.imwrite('picture/orgin_{}.jpg'.format(id),frame)  #保存原图
            img = cv2.cvtColor(numpy.asarray(image),cv2.COLOR_RGB2BGR)  
            cv2.imwrite('picture/recog_{}.jpg'.format(id),img)    #YOLO返回的图片
            
            for label in labels:                # 将返回的标签添加入labelCache
                labelCache.append(label)
            Recog_result = judge(labelCache)    # 将laelCache用于房间判断
            if Recog_result == '未识别到':            # 如果当前无法判断房间类型：添加判断数据
                sk.send(b"restart")

                last_room_state = room_state
                continue                        # 这一步没有清空所处房间的已识别信息
            ResultCache[room_state] = Recog_result    # 判断结果加入存储房间类型的列表末尾
            print(ResultCache)
            sk.send(b"finish")                  # 结束本次房间判断
            labelCache = []                     # 成功识别后清空该房间的识别信息
            continue                            # 重新返回待识别状态
        elif state == b'free':
            continue

        # 重启相机
        elif state==b'restart':
            camera.close()
            time.sleep(1)
            camera = Camera1920()

def judge(labelcahe):
    '''
    对本次图片识别结果进行判断，
    得出所处房间的类型。
    return '房间类型的字符串'
    '''
    # 神经网络训练时的对应标签
    # 桌椅chair-0,餐具fork-1,食物food-2，人person-3
    # 宠物pet-4,床bed-5,沙发sofa-6,电视televison-7
    
    if 1 in labelcahe:  #暂时修改
        return '餐厅'
    if 2 in labelcahe:
        return '餐厅'
    if 5 in labelcahe:
        return '卧室'
    if 6 in labelcahe:
        return '客厅'
    if 7 in labelcahe:
        return '客厅'
    # 处特定物品外，只存在人-3，宠物-4组合能判断房间类型
    if 3 in labelcahe:
        if 4 in labelcahe:
            return '卧室'
    return '未识别到' # 赌博游戏，记得更改

def check(ResultCache):
    """
    如果出现一个房间未识别的情况，
    可以通过排除法纠正
    """
    ResultDict = {'A':"未识别到" , "B":"未识别到","C":"未识别到"}
    room = ["A","B","C"]
    room_num = {"餐厅":0, "卧室":0, "客厅":0, "未识别到":0}
    count = 0
    room_flag = ""
    for room_type in ResultCache:
        if room_type == "未识别到":
            room_flag = room[count]
            print(room_flag)
        ResultDict[room[count]] = room_type
        room_num[room_type] += 1
        count += 1
    if room_num["未识别到"] == 1:
        for room_type in room_num:
            # 找到没有出现的房间类型
            if room_num[room_type] == 0:
                room_none = room_type
                # print(room_none)
        ResultDict[room_flag] = room_none
    print([ResultDict["A"], ResultDict["B"], ResultDict["C"]])
    return [ResultDict["A"], ResultDict["B"], ResultDict["C"]]
            
            
if __name__ == '__main__':
    main()

