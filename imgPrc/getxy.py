#导入opencv和pandas数据包
import cv2
import pandas as pd
# 导入图片
camare=cv2.VideoCapture(1)
ret,img=camare.read()
#img = cv2.imread("D:\\Pho\\p1.jpg")   #代码参数一般是没有问题的，一本如果不能实现在图上标点的话，基本上都是导图图片出错了
# 建立空列表存放像素坐标
a =[]
b = []


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    # 点击鼠标左键
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        a.append(x)
        b.append(y)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 0), thickness=1)
        cv2.imshow("image", img)


cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)
cv2.waitKey(0)


# 保存数据




