import cv2
import numpy as np

pts1 = np.float32([[243, 55], [408, 55], [93, 464], [501, 472]])
pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])
M = cv2.getPerspectiveTransform(pts1, pts2)
cram = cv2.VideoCapture(0)

while True:
    flg, src = cram.read()
    src=cv2.imread("D:\\Pho\\p1.jpg")
    res = cv2.warpPerspective(src, M, (int(300), int(300)))
    [b, g, r] = cv2.split(res)
    g = cv2.add(g, b)
    b = g
    r = cv2.add(r, b)
    res = cv2.merge([b, g, r])
    #res= cv2.GaussianBlur(res, (3,3), 0) 高斯滤波，适用于对噪点的清除
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 168, 255, cv2.THRESH_BINARY)#阈值根据实际情况调整
    cv2.imshow("H", binary)

    cv2.waitKey(1)
