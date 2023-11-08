import os
import numpy as np
import cv2
from Motor_Control import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

DXL = Motor_Controller()

CAMERANAME = 0  # Laptop webcam : 0(default value)
DEFAULT_VELOCITY = 200 # Need tuning

# Open camera
cap = cv2.VideoCapture(CAMERANAME)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

# PID Gain -> need tuning
Kp = 1.0
Ki = 0.5
Kd = 1.0

error_b = 0
error_i = 0

cX = 0
cY = 0

# Detect line
while True:
    ret, frame = cap.read()
    frame_lr = cv2.flip(frame,1)
    h,w = frame_lr.shape[:2]
    cv2.circle(frame_lr, (int(w/2),int(h/2)), 2, (0,255,255),-1)

    frame_gray = cv2.cvtColor(frame_lr,cv2.COLOR_BGR2GRAY)
    frame_blur = cv2.GaussianBlur(frame_gray,(5,5),0)
    ret, thresh1 = cv2.threshold(frame_blur,123,255,cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        cmax = max(contours,key=cv2.contourArea)
        cv2.drawContours(frame_lr, contours, -1, (0,255,255), 1 )
        M = cv2.moments(cmax)

        if M['m00']>0:
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(frame_lr, (cX, cY), 2, (0, 255, 255), -1)
    cv2.imshow('frame_lr', frame_lr) #orignal frame(fliped) + contours + centroid
    cv2.imshow('frame_thresh', thresh1)

    # PID control for velocity
    if cX > 0:
        error_p = w/2-cX
        error_i += error_p

        # To prevent error_i becoming too large
        if abs(error_p) <= 5:
            error_i = 0

        error_d = error_b-error_p
        error_control = Kp*error_p + Ki*error_i + Kd*error_d
        error_b = error_p
        print('error : %d       error_control : %d' %(error_p, error_control))

        # Call MotorController function
        # Both Motors' DRIVE_MODE : NORMAL_MODE(CCW : Positive, CW : Negative)
        DXL.MotorController(DXL.RIGHT_ID, -(DEFAULT_VELOCITY + error_control))
        DXL.MotorController(DXL.LEFT_ID, (DEFAULT_VELOCITY - error_control))

    if cv2.waitKey(10) == 27:
        break

# Close camera
cap.release()
cv2.destroyAllWindows()

# Disable torque on Motor & Close Port
DXL.Unconnect_Motor()