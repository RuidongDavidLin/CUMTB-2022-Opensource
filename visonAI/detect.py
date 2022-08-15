import time
import cv2
import numpy as np
from PIL import Image
from yolov4tiny.yolo import YOLO

class yolov4:
    def __init__(self):
        self.yolo=YOLO()

    def getLabels(self,frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = Image.fromarray(np.uint8(frame))
        data = self.yolo.detect_image(frame, isRetImg=False)
        return data['labels']

    def getLabelsAndImg(self,frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = Image.fromarray(np.uint8(frame))
        data = self.yolo.detect_image(frame, isRetImg=True)
        return data['labels'],data['img']

if __name__ == '__main__':

	try:
		yolo=yolov4()
		capture = cv2.VideoCapture(0)
		while (True):
		#t1 = time.time()
		# 读取某一帧
			ref, frame = capture.read()
			label,image=yolo.getLabelsAndImg(frame)
			print(label)
			img = cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
			cv2.imshow('yolo',img)

	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		exit(0)
