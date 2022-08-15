# 用于采集图像数据集

import cv2
from camera.camera import Camera1920
import time
#数据集采集种类,默认为meat
#数据集采集方式，默认为自动，0-自动——每隔1s自动拍照并存储，1-手动——每次按下enter键拍照并存储
Collection_method = 0
Collection_group = 0
def main():
    camera = Camera1920()
    frame = camera.get()
    #如果是绝对路径不能有中文和空格
    #存放到相对路径要提前建立好文件夹
    dir = Collection_type + '/' + Collection_type 
    i = (Collection_group - 1) * 10000  + Initial_num
    if Collection_method:
        #手动采集，按下回车即可采集
        while True:
            i += 1
            x = 0
            if input('输入回车键以采集一次数据：') == '':
                for x in range(6):
                    frame = camera.get()  # 采集一帧图片
                cv2.imwrite(dir + '%05d.jpg' %i, frame)  # 写入图片以及命名
                cv2.namedWindow("miku",0)
                cv2.resizeWindow("miku", 640, 480)
                cv2.imshow("miku", frame)
                print('%05d.采集成功' %i)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
    else:
        #自动采集
        End_num=int(input('End_num:'))
        while i< (Collection_group -1) * 10000 + End_num:
            i += 1
            x = 0
            for x in range(6):
                frame = camera.get()
            cv2.imwrite(dir + '%05d.jpeg' %i, frame)  # 写入图片以及命名
            cv2.namedWindow("miku", 0)
            cv2.resizeWindow("miku", 640, 480)
            cv2.imshow("miku", frame)
            print('%05d.采集成功' %i)
            cv2.waitKey(Image_showtime)
            time.sleep(0.01)  # 休眠一秒 可通过这个设置拍摄间隔，类似帧。

    camera.close()

if __name__ == '__main__':
    try:
        Collection_type=str(input('请输入采集类型:'))
        Collection_method=int(input('请输入采集方法（0-自动采集，1-手动采集）:'))
        Collection_group=int(input('colletion group:'))
        Initial_num = int(input('Initial_num:'))
        Image_showtime = int(input('Image_time:'))
        
        main()
    except KeyboardInterrupt:
        print("结束采集数据集")
