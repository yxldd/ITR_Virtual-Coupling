import cv2 as cv
import numpy as np

Kp = 30
Ki = 0
Kd = 0

class PID:
    def __init__(self, goal, kp, ki, kd):
        self.SetPoint = goal
        self.Proportion = kp
        self.Integral = ki
        self.Derivative = kd
        self.LastError = 0
        self.PrevError = 0
        self.SumError = 0


def PIDCalc(pp, NextPoint):
    Error = pp.SetPoint - NextPoint
    pp.SumError += Error
    DError = pp.LastError - pp.PrevError
    pp.PrevError = pp.LastError
    pp.LastError = Error
    return pp.Proportion * Error + pp.Integral * pp.SumError + pp.Derivative * DError

def get_turnval(nowservo,pos):
    val=PIDCalc(nowservo,pos)
    if val>103:
        return 103
    if val<67:
        return 67
    return  val

servo = PID(0, Kp, Ki, Kd)

file = open('D:\\Pho\\dist.txt', 'w')
src = cv.imread("D:\\Pho\\out.jpg")
ret, binary = cv.threshold(src, 174, 255, cv.THRESH_BINARY)
i = 299
j = 0
print(src.shape[0], src.shape[1])
while i >= 0:
    j = 0
    while j < 150:
        if src[i][150 + j][1] <= 10:
            turnval=PIDCalc(servo,j)
            file.write("y: " + str(i) + "    val: " + str(turnval) + " \n")
            break
        if src[i][150 - j][1] <= 10:
            turnval=PIDCalc(servo,-j)
            file.write("y: " + str(i) + "    val: " + str(turnval) + " \n")
            break
        j = j + 1
    i = i - 1
