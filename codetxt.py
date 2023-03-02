import rclpy
import socket
import threading
import cv2
import numpy as np
from rclpy.node import Node
from aemc_msgs.msg import MarkerArray   


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,                                               
            'markers',
            self.listener_callback,
            10)
        # self.subscription

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%d"' % msg.num)   
        print(msg.markers[0].pose.position.x) 
        print("!")


Kp = 0.2
Ki = 0
Kd = 0.2


def tcp_link(sock, addr,turnval,speed):
    print(addr)
    print('link create, from %s:%s' % addr)
    sock.send(b'%da%d\n'%(turnval,speed))
    #print('link close, from %s:%s' % addr)


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


def bh(val):
    if val > 90:
        return 90
    if val < 64:
        return 64
    val=int(val)
    return val


servo = PID(0, Kp, Ki, Kd)
#pts1 = np.float32([[255, 178], [325, 178], [145, 461], [378, 447]])
pts1 = np.float32([[229, 165], [458, 176], [9,450], [635,453]])
pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])
M = cv2.getPerspectiveTransform(pts1, pts2)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind(('192.168.3.185', 11411))

s.listen(5)

video=cv2.VideoCapture(0)

def sum_binary(binary, cnt):
    i = 299
    sum_value = 0
    while i >= 250:
        j = 0
        while j < 150:
            if binary[i][150 + j] <= 10:
                sum_value = sum_value + ( 2 * j) * (2 * cnt - 1)
                cnt = cnt - 1
                #cv2.circle(binary, (150 + j, i), 1, (255, 255, 255), 2)
                break
            if binary[i][150 - j] <= 10:
                sum_value = sum_value + (-2 * j) * (2 * cnt - 1)
                cnt = cnt - 1
                #cv2.circle(binary, (150 - j, i), 1, (255, 255, 255), 2)
                break
            j = j + 1
        i = i - 10
    return sum_value


class image_converter:
    while(video.isOpened()):
        ret,src=video.read()
        res = cv2.warpPerspective(src, M, (int(300), int(300)))
        [r, g, b] = cv2.split(res)
        g = cv2.add(g, b)
        b = g
        r = cv2.add(r, b)
        res = cv2.merge([b, g, r])
        cv2.imwrite("J.jpg",res)
        res = cv2.GaussianBlur(res, (3, 3), 0)  # 高斯滤波，适用于对噪点的清除
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # 阈值根据实际情况调整
        #cv2.imshow("H", binary)
        cv2.imwrite("H.jpg",binary)
        sum_value = sum_binary(binary, 5)
        sum_value = sum_value / 25
        turnval = PIDCalc(servo, sum_value)+77
        print(turnval)
        cv2.imwrite("TP.jpg",src)
        #sock,addr=s.accept()
        #print(sock,addr)
        t=threading.Thread(target=tcp_link,args=(sock,addr,bh(turnval),25))
        t.start()


def doit():
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    image_converter()
    rospy.spin()



if __name__ == "__main__":
    sock,addr=s.accept()
    #print(sock,addr)
    #t=threading.Thread(target=tcp_link,args=(sock,addr))
    #t.start()
    doit()
