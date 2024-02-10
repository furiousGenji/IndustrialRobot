import cv2
import numpy as np
import serial
import sympy
import coordiTrans
import robot
import time

worldFinalPos = np.array([[0], [0], [0], [0]])
s = serial.Serial('COM5', 115200, timeout=5)
waitingTime = 3

def read_usb_capture():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    while (cap.isOpened()):
        ret, frame= cap.read()
        frame =cv2.undistort(frame,coordiTrans.mtx,coordiTrans.dist,None,coordiTrans.mtx)
        if ret:
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # Gaussian Blur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # HSV image conversion
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # Erosion
            inRange_hsv = cv2.inRange(erode_hsv, np.array([35, 43, 35]), np.array([90, 255, 255]))
            cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                cv2.drawContours(frame, [np.intp(box)], -1, (0, 255, 255), 2)

                M = cv2.moments(c)  # Calculate the moments of each order of the first contour in the form of dictionary
                if M['m00'] != 0:
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00'])
                    angle = rect[2]
                    if angle >= 60:
                        angle = angle - 90

                    worldPos = coordiTrans.camera_to_world(center_x, center_y, 401,
                                                           robot.d2r(180), 0, robot.d2r(180), -10, 270, 400)
                    for e in worldPos:
                        print(float(e))

                    if cv2.waitKey(1) == 13:
                        worldFinalPos = worldPos
                        break

                    print('angle:', angle)
                    cv2.circle(frame, (center_x, center_y), 7, 128, -1) # Plot the center point
                    str1 = '(' + str(center_x) + ',' + str(center_y) + ',' + str(angle) + ')'  #Convert coordinate to strings
                    cv2.putText(frame, str1, (center_x - 50, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0),
                                2,cv2.LINE_AA)  # Plot coordinate point
            cv2.imshow("Image", frame)

    if worldFinalPos[0,0]>20:
        worldFinalPos[0, 0] += 30
        worldFinalPos[1, 0] -= 20
        worldFinalPos[2, 0] -= 0
    elif worldFinalPos[0,0]<-40:
        worldFinalPos[0, 0] -= 30
        worldFinalPos[1, 0] -= 10
        worldFinalPos[2, 0] -= 0
    else:
        worldFinalPos[0, 0] -= 10
        worldFinalPos[1, 0] -= 0
        worldFinalPos[2, 0] -= 0

    vector = robot.BraccioInverse(
        [0, worldFinalPos[0, 0], worldFinalPos[1, 0], worldFinalPos[2, 0], 0, 0, -sympy.exp(robot.qn / robot.pi)])

    command1 = robot.genBraccioString(vector, 10, angle)
    if command1 != -1:
        tempCommand = "P90,90,90,90,90,10,30"
        s.write(tempCommand.encode('UTF-8'))
        time.sleep(waitingTime)
        s.write(command1.encode('UTF-8'))

    vectorPi = vector * 180 / robot.pi
    print(sympy.N(robot.BraccioForward(vectorPi[0], vectorPi[1], vectorPi[2], vectorPi[3], vectorPi[4])))
    print(s.readline().decode())
    print(command1)
    time.sleep(waitingTime)
    command2 = robot.genBraccioString(robot.BraccioInverse([0, worldFinalPos[0, 0], worldFinalPos[1, 0], worldFinalPos[2, 0], 0, 0, -sympy.exp(robot.qn / robot.pi)]), 100, angle)
    command3 = robot.genBraccioString(robot.BraccioInverse([0, worldFinalPos[0, 0]+10, worldFinalPos[1, 0]-60, worldFinalPos[2, 0]+80, 0, 0, -sympy.exp(robot.qn / robot.pi)]), 100,angle)
    command4 = robot.genBraccioString(robot.BraccioInverse([0, -215, 0, 90, 0, 0, -sympy.exp(robot.qn / robot.pi)]), 100, angle)
    command5 = robot.genBraccioString(robot.BraccioInverse([0, -215, 0, 90, 0, 0, -sympy.exp(robot.qn / robot.pi)]), 20, angle)
    s.write(command2.encode('UTF-8'))
    time.sleep(waitingTime)
    s.write(command3.encode('UTF-8'))
    time.sleep(waitingTime)
    s.write(command4.encode('UTF-8'))
    time.sleep(waitingTime)
    s.write(command5.encode('UTF-8'))
    time.sleep(waitingTime)

    cap.release()

if __name__ == '__main__':
    read_usb_capture()
