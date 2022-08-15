#coding=utf-8
import cv2
import numpy
from multiprocessing import Process,Queue
import time


class Camera1920:
    def __init__(self):
        self.capture= cv2.VideoCapture(0)
        self.capture.set(3, 1920)
        self.capture.set(4, 1080)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
    def get(self):
        ret, frame=self.capture.read()
        return frame
    def close(self):
        self.capture.release()

class Camera1280:
    def __init__(self):
        self.capture= cv2.VideoCapture(0)
        self.capture.set(3, 1280)
        self.capture.set(4, 720)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
    def get(self):
        ret, frame=self.capture.read()
        return frame
    def close(self):
        self.capture.release()

class Camera800:
    def __init__(self):
        self.capture= cv2.VideoCapture(0)
        self.capture.set(3, 800)
        self.capture.set(4, 600)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
        ret, frame=self.capture.read()
    def get(self):
        ret, frame=self.capture.read()
        return frame
    def close(self):
        self.capture.release()


class CameraTest:
    def __init__(self,fps):
        self.imgQueue=Queue()
        self.commandQueue=Queue()
        self.process=Process(target=self.update,args=(fps,self.imgQueue,self.commandQueue,))
        self.process.start()


    def update(self,fps,imgQueue,commandQueue):
        capture = cv2.VideoCapture('v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)1920, height=(int)1080, framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink', cv2.CAP_GSTREAMER)
        # capture = cv2.VideoCapture(0)
        capture.set(3, 1920)
        capture.set(4, 1080)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        commandQueue.get()
        while True:
            try:
                commandQueue.get_nowait()
                imgQueue.put(False)
                break
            except:
                ...
            ret,frame=capture.read()
            frame = cv2.flip(frame, 1)
            imgQueue.put(frame)
            time.sleep(1.0/fps)

    def get(self):
        return self.imgQueue.get()

    def empty(self):
        return self.imgQueue.empty()

    def start(self):
        self.commandQueue.put('start')

    def close(self):
        self.commandQueue.put('close')



if __name__ == '__main__':
    miku=Camera()
    while True:
        cv2.imshow('windows',miku.get())
        cv2.waitKey(1)


