import cv2
import numpy as np
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

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


def get_turnval(nowservo, pos):
    val = PIDCalc(nowservo, pos)
    if val > 103:
        return 103
    if val < 67:
        return 67
    return val

def callback(data):
    #cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

servo = PID(0, Kp, Ki, Kd)
pts1 = np.float32([[255, 178], [325, 178], [145, 461], [378, 447]])
pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])
M = cv2.getPerspectiveTransform(pts1, pts2)

if __name__ == '__main__':
    import sys 
    print(sys.version) # 查看python版本
    rospy.init_node('img_process_node', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/image_view/image_raw', Image, callback)
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

    while True:
        #src = cv2.imread("D:\\Pho\\p1.jpg")
        res = cv2.warpPerspective(cv_img, M, (int(300), int(300)))
        [b, g, r] = cv2.split(res)
        g = cv2.add(g, b)
        b = g
        r = cv2.add(r, b)
        res = cv2.merge([b, g, r])
        res = cv2.GaussianBlur(res, (3, 3), 0)  # 高斯滤波，适用于对噪点的清除
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 168, 255, cv2.THRESH_BINARY)  # 阈值根据实际情况调整
        # cv2.imshow("H", binary)
        i = 299
        sum = 0
        cnt = 5
        while i >= 250:
            j = 0
            while j < 150:
                if binary[i][150 + j] <= 10:
                    sum = sum + (- 2 * j) * (2 * cnt - 1)
                    cnt = cnt - 1
                    cv2.circle(binary, (150 + j, i), 1, (255, 255, 255), 2)
                    break
                if binary[i][150 - j] <= 10:
                    sum = sum + (2 * j) * (2 * cnt - 1)
                    cnt = cnt - 1
                    cv2.circle(binary, (150 - j,i), 1, (255, 255, 255), 2)
                    break
                j = j + 1
            i = i - 10
        sum = sum / 25
        turnval = PIDCalc(servo, sum)
        print(turnval)
        #cv2.imshow("H", binary)
       #cv2.waitKey(1)
        rospy.spin()