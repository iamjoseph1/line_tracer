import numpy as np
import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

Kp = 1.0
Ki = 1.0
Kd = 1.0

error_i = 0
error_b = 0
cX = 0
cY = 0

while True:
    ret, frame = cap.read()
    frame_lr = cv2.flip(frame,1) #화면 좌우반전( 0 : 상하반전, 1 : 좌우반전)
    #fps = round(cap.get(cv2.CAP_PROP_FPS)) #video의 초당 프레임 수
    br = 0 #영상의 밝기 조절 변수
    frame_lr_shine = cv2.add(frame_lr,(br,br,br,0))

    h,w = frame_lr_shine.shape[:2]
    cv2.circle(frame_lr_shine, (int(w/2),int(h/2)), 2, (0,255,255),-1)

    frame_hsv = cv2.cvtColor(frame_lr_shine,cv2.COLOR_BGR2HSV) #영상을 다루기 쉽게 BGR -> HSV
    mask_red = cv2.inRange(frame_hsv, (160,128,128), (180,255,255)) #HSV순서-빨간색 검출, S(선명도)를 조작해 살색 검출 제한
    #ㄴinRange함수를 사용한 mask영상은 이진화 영상이다.
    frame_red = cv2.bitwise_and(frame_lr_shine,frame_lr_shine,mask = mask_red)
    #ㄴ원본 영상과 mask에 대해 and연산을 수행해 1인 곳만(빨간색인 곳만) 살리고 나머지는 0(검은색)으로 표시 
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        print('number of contour : {}'.format(len(contours)))
        cmax = max(contours,key=cv2.contourArea)
        cv2.drawContours(frame_lr_shine, contours, -1, (0,255,255), 1 )
        M = cv2.moments(cmax)

        #if cv2.contourArea(cmax) > 0: #cmax contour의 넓이가 0보다 큰 경우(M['m00'] > 0)
        if M['m00']>0:
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(frame_lr_shine, (cX, cY), 2, (0, 255, 255), -1)
            print('center_of_red : ({x},{y})'.format(x=cX,y=cY))

    #PID control for motor_rpm
    if cX > 0:
        error_p = w/2-cX
        error_i += error_p
        error_d = error_b-error_p
        print('error : {}'.format(error_p))

        error_control = Kp*error_p + Ki*error_i + Kd*error_d
        print('motor_rpm : {}'.format(error_control))
        error_b = error_p


    cv2.imshow('frame_lr', frame_lr_shine) #contour와 무게중심이 덧입혀진 원본 영상
    #cv2.imshow('frame_red', frame_red) #빨간색 객체만 나타나는 영상(배경은 검은색)

    if cv2.waitKey(10) == 27:
        break

cap.release()
cv2.destroyAllWindows()