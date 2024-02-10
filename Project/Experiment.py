import serial
import time
import sympy

def DHTransformation(d,delta,a,alpha):
    return Matrix([cos(delta),-sin(delta)*cos(alpha),sin(delta)*sin(alpha),a*cos(delta)],
             [sin(delta),cos(delta)*cos(alpha),-cos(delta)*sin(alpha),a*sin(delta)],
             [0,sin(alpha),cos(alpha),d],
             [0,0,0,1])


def BraccioForward(base,shoulder,elbow,wrist,twist):
    return None
print(cos(90))

    
# def TestBraccioForward(base,shoulder,elbow,wrist,twist):
#
#     s = serial.Serial('COM4', 115200, timeout=5)
#     time.sleep(3)
#     print(s.readline().decode())
#     s.write(b'P0,90,180,90,0,100,30')
