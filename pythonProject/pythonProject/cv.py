# time: 2022/3/21 15:19]
import cv2
img=cv2.imread('/home/nao/recordings/cameras/image.jpg')
frameHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)