import cv2 as cv
import numpy as np
file = open('D:\\Pho\\dist.txt','a')
src=cv.imread("D:\\Pho\\out.jpg")
ret, binary = cv.threshold(src, 174, 255, cv.THRESH_BINARY)
i=0
j=0
print(src.shape[0],src.shape[1])
while i<300:
    j=0
    str1=""
    while j<300:
        str1=str1+str(src[i][j][1])+" "
        j=j+1
    file.write(str1+'\n')
    i=i+1



