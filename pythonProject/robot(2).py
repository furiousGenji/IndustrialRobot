import math

import serial
import time
from sympy import *

#Connector
s = serial.Serial('COM3', 115200, timeout=5)
time.sleep(3)

# 机械臂各个部分长度
d1 = 70
a2 = 125
a3 = 125
a4 = 90
d5 = 150

# 逆向运动学数据，qn是tcp绕z轴旋转角度，xyz是tcp世界坐标
qn = 0
x = 215
y = 0
z = 1


def d2r(deg):
    return deg * pi / 180


# DH变换矩阵
def DHTransformation(d, delta, a, alpha):
    return Matrix([[cos(delta), -sin(delta) * cos(alpha), sin(delta) * sin(alpha), a * cos(delta)],
                   [sin(delta), cos(delta) * cos(alpha), -cos(delta) * sin(alpha), a * sin(delta)],
                   [0, sin(alpha), cos(alpha), d],
                   [0, 0, 0, 1]])


# 正向运动学
def BraccioForward(base, shoulder, elbow, wrist, twist):
    temp = DHTransformation(d1, d2r((base + 180)), 0, pi / 2) \
           * DHTransformation(0, d2r(shoulder) , a2, 0) \
           * DHTransformation(0, d2r((elbow - 90)), a3, 0) \
           * DHTransformation(0, d2r((wrist - 90)), a4, -pi / 2) \
           * DHTransformation(d5, d2r((twist - 90)), 0, 0)
    return temp * Matrix([0, 0, 0, 1])


# 正向运动学
def TestBraccioForward(base, shoulder, elbow, wrist, twist):
   # print(s.readline().decode())
    matrix = BraccioForward(base, shoulder, elbow, wrist, twist)
    print("正向运动学结果：")
    print(N(matrix))  # 给定角度计算tcp的世界坐标
    # s.write(b'P0,90,180,90,0,100,30')
    s.write(str('P' + str(int(base)) + ','
                + str(int(shoulder)) + ','
                + str(int(elbow)) + ','
                + str(int(wrist)) + ','
                + str(int(twist)) + ','
                + '90,30').encode('utf-8'))  # 必须是integer
    #print(s.readline().decode())


# 逆向运动学
def BraccioInverse(w):
    q1 = atan2(w[2], w[1])
    # print(q1)
    q234 = atan2(-w[4] * cos(q1) - w[5] * sin(q1), -w[6])
    b1 = w[1] * cos(q1) + w[2] * sin(q1) - a4 * cos(q234) + d5 * sin(q234)
    b2 = d1 - a4 * sin(q234) - d5 * cos(q234) - w[3]
    b_pow2 = math.pow(b1, 2) + math.pow(b2, 2)
    q3 = acos((b_pow2 - math.pow(a2, 2) - math.pow(a3, 2)) / (2 * a2 * a3))
    # print("q3="+str(q3))
    q2 = atan2((a2 + a3 * cos(q3)) * b2 - a3 * b1 * sin(q3), (a2 + a3 * cos(q3)) * b1 + a3 * b2 * sin(q3))
    q4 = q234 - q2 - q3
    q5 = q1 + pi * log(sqrt(math.pow(w[4], 2) + math.pow(w[5], 2) + math.pow(w[6], 2)))
    q5 %= pi

    #Adapt the offset
    q2 += pi
    q3 += pi / 2
    q4 += pi / 2

    print("逆向运动学结果：")
    print(N(Matrix([q1, q2, q3, q4, q5])))  # 给定tcp世界坐标与角度，计算逆向运动学各节点的角度(弧度制)
    return Matrix([q1, q2, q3, q4, q5])


def genBraccioString(q):
    qtemp = q * (180 / pi)
    # generate the robot string
    print(qtemp[0],qtemp[1],qtemp[2],qtemp[3],qtemp[4])
    go = True
    for angle in qtemp:
        # print(angle)
        try:  # if something goes wrong, the script won't stop
            if not 0 <= angle <= 180:
                # print(angle)
                go = False
                break
        except:
            go = False
            break
    if go:
        command = "P" + str(int(qtemp[0])) + "," \
                  + str(int(qtemp[1])) + "," \
                  + str(int(qtemp[2])) + "," \
                  + str(int(qtemp[3])) + "," \
                  + str(int(qtemp[4])) + ",100,50\n"
        # print(angles)
    else:
        # print("not in range")
        command = None
    # return command
    # print(command)
    return qtemp

# s.write("P90,90,90,0,90,10,30".encode("UTF-8"))
qVector = genBraccioString(BraccioInverse([0, x, y, z, 0, 0, -exp(qn / pi)]))  # 数组第一个值没有用
print("逆向运动学结果角度制：")
print(N(qVector))  # BraccioInverse结果转角度制
TestBraccioForward(qVector[0], qVector[1], qVector[2], qVector[3], qVector[4])

#老师用来判断边界的代码
print("Start iteration")
posx= []
posy= []
for x in range(-270,270,30): #-270,270,50):
    for y in range(-270,+270,30): #-300,300,50):
        w=Matrix([0,x,y,80,0,0,-exp(1/2)])
        angles=genBraccioString(BraccioInverse(w))
        print("x="+str(x)+" " + "y=" + str(y) , end=" ")
        if angles:
            posx.append(x)
            posy.append(y)
            s.write(angles.encode('UTF-8'))
            s.readline()
            print(": " + angles, end=" ")
        else:
            print(": ---")
print("Success")