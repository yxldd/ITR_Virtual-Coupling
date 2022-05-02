import cv2 as cv
import numpy as np

file = open('D:\\Pho\\dist.txt', 'w')
src = cv.imread("D:\\Pho\\out.jpg")
ret, binary = cv.threshold(src, 174, 255, cv.THRESH_BINARY)
i = 299
j = 0
print(src.shape[0], src.shape[1])
while i >= 0:
    j = 0
    while j < 150:
        if (src[i][150 + j][1] <= 10):
            file.write("y: " + str(i) + "    pos: " + str(150 + j) + " \n")
            break
        if (src[i][150 - j][1] <= 10):
            file.write("y: " + str(i) + "    pos: " + str(150 - j) + " \n")
            break
        j = j + 1
    i = i - 1
