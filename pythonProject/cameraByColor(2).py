import cv2
import argparse
import imutils
import numpy as np

def read_usb_capture():
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)

    #cv2.namedWindow('real_img',cv2.WINDOW_NORMAL)
    while(cap.isOpened()):

        ret, frame=cap.read()
        if ret:
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # 腐蚀 粗的变细
            inRange_hsv = cv2.inRange(erode_hsv, np.array([35, 43, 35]), np.array([90, 255, 255]))
            cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)


                M = cv2.moments(c)  # 计算第一轮廓的各阶矩,字典形式
                if M['m00']!=0:
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00'])
                    angle = rect[2]
                    print('center_x:', center_x)
                    print('center_y:', center_y)
                    print('angle:', angle)
                    cv2.circle(frame, (center_x, center_y), 7, 128, -1)  # 绘制中心点
                    str1 = '(' + str(center_x) + ',' + str(center_y) + ',' + str(angle) + ')'  # 把坐标转化为字符串
                    cv2.putText(frame, str1, (center_x - 50, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2,
                                cv2.LINE_AA)  # 绘制坐标点位
            cv2.imshow("Image", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()

if __name__=='__main__':
    read_usb_capture()
