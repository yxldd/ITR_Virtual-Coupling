#导入opencv和pandas数据包
import cv2
import pandas as pd


# 导入图片
img = cv2.imread('D:\\Pho\\ca.jpg')   #代码参数一般是没有问题的，一本如果不能实现在图上标点的话，基本上都是导图图片出错了
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

# print(a[0],b[0])

# 保存数据
data = [a,b]
data = pd.DataFrame(data, index=['h', 'l'] )

#像素坐标保存，r'C:\Users\ASUS\Desktop\ccnu.csv'位置
data.to_csv(r'C:\Users\ASUS\Desktop\ccnu.csv')  #注意建立的存储像素坐标的格式表格是csv,不是excel默认格式xlsx，否则数据是无妨保存，不信可以试试看。“把这行的csv,改成xlsx格式
#注意点像素坐标时，只能是一次，下次再点时原来的数据就会消失，要特别注意

# if __name__ == '__main__':
#导入opencv和pandas数据包
import cv2
import pandas as pd


# 导入图片
img = cv2.imread('D:\\Pho\\ca.jpg')   #代码参数一般是没有问题的，一本如果不能实现在图上标点的话，基本上都是导图图片出错了
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

# print(a[0],b[0])

# 保存数据
data = [a,b]
data = pd.DataFrame(data, index=['h', 'l'] )

#像素坐标保存，r'C:\Users\ASUS\Desktop\ccnu.csv'位置
data.to_csv(r'D:\\Pho\\carema.csv')
matrix = get_matrix(img, pts)
output = cv2.warpPerspective(img, matrix, (int(640), int(480 + 50)))
