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

CAMERANAME = "/dev/video0" # Laptop webcam : 0(default value)
DEFAULT_VELOCITY = 100 # Need tuning

# Open camera
cap = cv2.VideoCapture(CAMERANAME, cv2.CAP_V4L)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

# PID Gain -> need tuning
Kp = 0.5
Ki = 0
Kd = 0

error_b = 0
error_i = 0

cX = 0
cY = 0

# Detect line & red object
while True:

    # Detect line
    ret, frame = cap.read()
    frame_lr = cv2.flip(frame,1)
    h,w = frame_lr.shape[:2]
    cv2.circle(frame_lr, (int(w/2),int(h/2)), 2, (0,255,255),-1)

    frame_gray = cv2.cvtColor(frame_lr,cv2.COLOR_BGR2GRAY)
    frame_blur = cv2.GaussianBlur(frame_gray,(5,5),0)
    ret, thresh1 = cv2.threshold(frame_blur,123,255,cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Detect red object
    img_hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([170,50,50]) #example value
    upper_red = np.array([180,255,255]) #example value
    mask_red = cv2.inRange(img_hsv, lower_red, upper_red)
    red_contours, red_ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    

    if len(contours) > 0:
        cmax = max(contours,key=cv2.contourArea)
        cv2.drawContours(frame_lr, contours, -1, (0,255,255), 1 )
        M = cv2.moments(cmax)

        if M['m00']>0:
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(frame_lr, (cX, cY), 2, (0, 255, 255), -1)

            error_p = w/2-cX
            error_i += error_p

            # To prevent error_i becoming too large
            if abs(error_p) <= 5:
                error_i = 0

            error_d = error_b-error_p
            error_control = Kp*error_p + Ki*error_i + Kd*error_d
            error_b = error_p
            print('error : %d       error_control : %d' %(error_p, error_control))
            
            left_vel  = int(DEFAULT_VELOCITY + error_control)
            right_vel = int(-(DEFAULT_VELOCITY - error_control))
            print('left_vel : %d       right_vel : %d' %(left_vel, right_vel))
            DXL.Dual_MotorController(left_vel, right_vel)

            if len(red_contours) != 0:
                print('red detected, STOP')
                DXL.Dual_MotorController(0, 0)
                time.sleep(0.5)
                DXL.Dual_MotorController(100, -100)
                time.sleep(3)                               # we need delay to escape from red object
                continue                                    # if we use 'break' instead of 'continue', loop will be stopped..!

        else :
            DXL.Dual_MotorController(0,0)

    cv2.imshow('frame_lr', frame_lr) #orignal frame(fliped) + contours + centroid
    cv2.imshow('frame_thresh', thresh1)
    cv2.imshow('mask_red', mask_red)

    # # PID control for velocity
    # if cX > 0:
    #     error_p = w/2-cX
    #     error_i += error_p

    #     # To prevent error_i becoming too large
    #     if abs(error_p) <= 5:
    #         error_i = 0

    #     error_d = error_b-error_p
    #     error_control = Kp*error_p + Ki*error_i + Kd*error_d
    #     error_b = error_p
    #     print('error : %d       error_control : %d' %(error_p, error_control))

    #     # Call MotorController function
    #     # Both Motors' DRIVE_MODE : NORMAL_MODE(CCW : Positive, CW : Negative)
    #     print('right : ',int(-(DEFAULT_VELOCITY + error_control)))
    #     print('LEFT : ', int(DEFAULT_VELOCITY - error_control))
    #     # DXL.MotorController(DXL.RIGHT_ID, int(-(DEFAULT_VELOCITY + error_control)))
    #     # DXL.MotorController(DXL.LEFT_ID, int(DEFAULT_VELOCITY - error_control))
    #     DXL.MotorController(DXL.RIGHT_ID, int(error_p))
    #     DXL.MotorController(DXL.LEFT_ID, -int(error_p))


    if cv2.waitKey(10) == ord('q'):
        DXL.Dual_MotorController(0,0)
        break

# Close camera
cap.release()
cv2.destroyAllWindows()

# Disable torque on Motor & Close Port
DXL.Unconnect_Motor()
