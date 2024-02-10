import math
from sympy import *

# Length of each part of the robot arm
d1 = 70
a2 = 125
a3 = 125
a4 = 90
d5 = 150
qn = 0

def d2r(deg):
    return deg * pi / 180


# DH Transformation
def DHTransformation(d, delta, a, alpha):
    return Matrix([[cos(delta), -sin(delta) * cos(alpha), sin(delta) * sin(alpha), a * cos(delta)],
                   [sin(delta), cos(delta) * cos(alpha), -cos(delta) * sin(alpha), a * sin(delta)],
                   [0, sin(alpha), cos(alpha), d],
                   [0, 0, 0, 1]])


# Forward Kinematic
def BraccioForward(base, shoulder, elbow, wrist, twist):
    temp = DHTransformation(d1, d2r((base + 180)), 0, pi / 2) \
           * DHTransformation(0, d2r(shoulder), a2, 0) \
           * DHTransformation(0, d2r((elbow - 90)), a3, 0) \
           * DHTransformation(0, d2r((wrist - 90)), a4, -pi / 2) \
           * DHTransformation(d5, d2r((twist - 90)), 0, 0)
    return temp * Matrix([0, 0, 0, 1])


# Inverse Kinematic, w[1,2,3] is the position of the gripper in KS0, w[4,5,6] is the rotation angle around x y z in the KS0
# Result is in radius
def BraccioInverse(w):
    q1 = atan2(w[2], w[1])
    q234 = atan2(-w[4] * cos(q1) - w[5] * sin(q1), -w[6])
    b1 = w[1] * cos(q1) + w[2] * sin(q1) - a4 * cos(q234) + d5 * sin(q234)
    b2 = d1 - a4 * sin(q234) - d5 * cos(q234) - w[3]
    b_pow2 = math.pow(b1, 2) + math.pow(b2, 2)
    q3 = acos((b_pow2 - math.pow(a2, 2) - math.pow(a3, 2)) / (2 * a2 * a3))
    q2 = atan2((a2 + a3 * cos(q3)) * b2 - a3 * b1 * sin(q3), (a2 + a3 * cos(q3)) * b1 + a3 * b2 * sin(q3))
    q4 = q234 - q2 - q3
    q5 = q1 + pi * log(sqrt(math.pow(w[4], 2) + math.pow(w[5], 2) + math.pow(w[6], 2)))
    q5 %= pi

    #Adapt the offset
    q2 += pi
    q3 += pi / 2
    q4 += pi / 2

    return Matrix([q1, q2, q3, q4, q5])


def genBraccioString(q, tcpAngular, offset):
    qtemp = q * (180 / pi)
    tempCommand = "P" + str(int(qtemp[0])) + "," \
                  + str(int(qtemp[1])) + "," \
                  + str(int(qtemp[2])) + "," \
                  + str(int(qtemp[3])+10) + "," \
                  + str(int(qtemp[4]) + offset + 10) + "," + str(int(tcpAngular)) + ",30\n"
    # generate the robot string
    go = True
    for angle in qtemp:
        try:  # if something goes wrong, the script won't stop
            if not 0 <= angle <= 270:
                go = False
                break
        except:
            go = False
            break
    if go:
        command = tempCommand
        return command
    else:
        print("not in range")
        return -1

