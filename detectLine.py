import os
import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

#set start time for timer
#start_time = time.time()

Kp = 1.0
Ki = 0.5
Kd = 1.0

error_b = 0
error_i = 0

cX = 0
cY = 0

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
    cv2.imshow('frame_lr', frame_lr) # orignal frame + contours + centroid
    cv2.imshow('frame_thresh', thresh1)

    #timer for reset error_i
    #if time.time()-start_time >= 0.1:
    #    error_i = 0
    #    start_time = time.time()

    #PID control for motor_rpm
    if cX > 0:
        error_p = w/2-cX
        error_i += error_p
        error_d = error_b-error_p
        error_control = Kp*error_p + Ki*error_i + Kd*error_d
        print('error : %d       error_control : %d' %(error_p, error_control))
        error_b = error_p
        
        #방향을 잡으면 에러 누적값을 초기화함
        if abs(error_p) <= 5:
            error_i = 0

    if cv2.waitKey(10) == 27:
        break

# Close camera
cap.release()
cv2.destroyAllWindows()
