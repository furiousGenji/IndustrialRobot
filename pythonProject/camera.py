import cv2
from shapedetector import ShapeDetector
from colorlabeler import ColorLabeler
import argparse
import imutils
import numpy as np


def GausBlur(image):
    dst = cv2.GaussianBlur(image, (5, 5), 0)
    #cv2.imshow('blur', dst)
    return dst

def Gray_img(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #增强对比度
    # gray=cv2.equalizeHist(gray)
    #消除阴影
    # average=int(np.mean(gray[gray>140]))
    # gray[gray<50]=average
    #cv2.imshow('gray', gray)
    return gray

def threshold_img(image):
    #设定二值化的界限值
    ret, binary = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    print("threshold value %s"%ret)
    cv2.imshow('threshold', binary)
    return binary

def open_mor(image):
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel, iterations=5)
    cv2.imshow('open', opening)
    return opening

def draw_shape(open_img,image):
    contours,hierarchy = cv2.findContours(open_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]

    rect=cv2.minAreaRect(cnt)
    box=cv2.boxPoints(rect)
    box= np.int0(box)
    cv2.drawContours(image,[box],0,(0,0,255),3)
    cv2.imshow('shape',image)

def read_usb_capture():
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)

    #cv2.namedWindow('real_img',cv2.WINDOW_NORMAL)
    while(cap.isOpened()):

        ret, frame=cap.read()
        if ret:
            gaus_img = GausBlur(frame)
            gray_img = Gray_img(gaus_img)
            thres_img = threshold_img(gray_img)
            open_img = open_mor(thres_img)
            draw_shape(open_img,frame)



        #cv2.imshow('real_img',frame)

        if cv2.waitKey(1)&0xFF==ord("q"):
            break

    cap.release()
    # cv2.destroyWindows()

if __name__=='__main__':
    read_usb_capture()