import cv2
import math
import socket

def segmentationXF(frame,det):
    k=0.35
    boxes= det[:4]
    lenth=int(math.ceil(boxes[2]-boxes[0])*4.5)
    centerX=(boxes[2]+boxes[0])/2
    centerY = (boxes[3] + boxes[1]) / 2
    top=int(centerY-k*lenth)
    bottom=int(centerY+(1-k)*lenth)
    left=int(centerX-0.5*lenth)
    right = int(centerX + 0.5 * lenth)
    topBorder=0
    bottomBorder=0
    leftBorder=0
    rightBorder=0
    if top<0:
        topBorder=0-top
        top = 0
    if left<0:
        leftBorder=0-left
        left = 0
    if bottom>=frame.shape[0]:
        bottomBorder=bottom-frame.shape[0]+1
        bottom=frame.shape[0]-1
    if right>=frame.shape[1]:
        rightBorder=right-frame.shape[1]+1
        right=frame.shape[1]-1
    frameClip=frame[top:bottom,left:right]
    frameClip = cv2.copyMakeBorder(frameClip, topBorder, bottomBorder, leftBorder, rightBorder, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    res=cv2.resize(frameClip,(224,224),interpolation=cv2.INTER_LINEAR)
    # cv2.rectangle(frame, (int(boxes[0]), int(boxes[1])+40), (int(boxes[2]), int(boxes[3])), (2, 255, 0), 1)
    # # 0左眼 2右眼
    # for i in range(0, 5):
    #     cv2.circle(frame, (int(lm[i * 2]), int(lm[i * 2 + 1])), 2, (0, 0, 255), -1)
    return res

def segmentation(frame,det):
    k=0.45
    boxes= det[:4]
    lenth=int(math.ceil(boxes[2]-boxes[0])*2.5)
    frame=cv2.copyMakeBorder(frame, lenth, lenth, lenth, lenth,cv2.BORDER_CONSTANT,value=(0, 0, 0))
    centerX=(boxes[2]+boxes[0])/2+lenth
    centerY = (boxes[3] + boxes[1]) / 2+lenth
    top=int(centerY-k*lenth)
    bottom=int(centerY+(1-k)*lenth)
    left=int(centerX-0.5*lenth)
    right = int(centerX + 0.5 * lenth)
    frameClip=frame[top:bottom,left:right]
    res=cv2.resize(frameClip,(224,224),interpolation=cv2.INTER_LINEAR)
    # cv2.rectangle(frame, (int(boxes[0]), int(boxes[1])+40), (int(boxes[2]), int(boxes[3])), (2, 255, 0), 1)
    # # 0左眼 2右眼
    # for i in range(0, 5):
    #     cv2.circle(frame, (int(lm[i * 2]), int(lm[i * 2 + 1])), 2, (0, 0, 255), -1)
    return res

def segmentationFace(frame,det):
    k = 0.35
    boxes = det[:4]
    lenth = int(math.ceil(boxes[2] - boxes[0]))
    frame = cv2.copyMakeBorder(frame, lenth, lenth, lenth, lenth, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    centerX = (boxes[2] + boxes[0]) / 2 + lenth
    centerY = (boxes[3] + boxes[1]) / 2 + lenth
    top = int(centerY - lenth*k)
    bottom = int(centerY + lenth*k)
    left = int(centerX - k * lenth)
    right = int(centerX + k * lenth)
    frameClip = frame[top:bottom, left:right]
    res = cv2.resize(frameClip, (224, 224), interpolation=cv2.INTER_LINEAR)
    # cv2.rectangle(frame, (int(boxes[0]), int(boxes[1])+40), (int(boxes[2]), int(boxes[3])), (2, 255, 0), 1)
    # # 0左眼 2右眼
    # for i in range(0, 5):
    #     cv2.circle(frame, (int(lm[i * 2]), int(lm[i * 2 + 1])), 2, (0, 0, 255), -1)
    return res


def lapulase(frame):
    img2gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 将图片压缩为单通道的灰度图
    res = cv2.Laplacian(img2gray, cv2.CV_64F)
    score = res.var()
    return score

def equalizeHist(frame):
    frame=frame.copy()
    ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR, frame)
    return frame

def boxExpend(dets,h,w):
    expandDets = dets.copy()
    for i in range(len(dets)):
        lenth = (dets[i][3] - dets[i][1]) * 1
        expandDets[i][0] = dets[i][0] - lenth if dets[i][0] - lenth > 0 else 0
        expandDets[i][1] = dets[i][1] - lenth if dets[i][1] - lenth > 0 else 0
        expandDets[i][2] = dets[i][2] + lenth if dets[i][2] + lenth < w else w - 1
        expandDets[i][3] = dets[i][3] + lenth if dets[i][3] + lenth < w else w - 1
        expandDets[i][4] = i
    return expandDets

def getSocket():
    s = socket.socket()  # 创建 socket 对象
    port = 39393  # 设置端口
    s.bind(('127.0.0.1', port))  # 绑定端口
    # s.listen(1)中1表示服务器允许排队数
    s.listen(1)
    # s.accept()表示接受一个来自客户端的连接请求
    c, addr = s.accept()
    return c