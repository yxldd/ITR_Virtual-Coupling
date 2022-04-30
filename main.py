import cv2
import numpy as np

pts1 = np.float32([[279, 268], [374, 270], [228, 395], [390, 403]])
pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])
M = cv2.getPerspectiveTransform(pts1, pts2)
cram = cv2.VideoCapture(0)

while True:
    flg, src = cram.read()
    res = cv2.warpPerspective(src, M, (int(300), int(300)))
    [b, g, r] = cv2.split(res)
    g = cv2.add(g, b)
    b = g
    r = cv2.add(r, b)
    res = cv2.merge([b, g, r])
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 174, 255, cv2.THRESH_BINARY)
    cv2.imshow("H", src)
    cv2.waitKey(1)
