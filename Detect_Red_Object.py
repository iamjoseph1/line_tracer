import numpy as np
import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

while True:
    ret, frame = cap.read()
    frame_lr = cv2.flip(frame,1) # flip image( 0 : up-down, 1 : left-right)
    # fps = round(cap.get(cv2.CAP_PROP_FPS)) #frame per second

    br = 0 # parameter for brightness adjustment
    frame_lr_shine = cv2.add(frame_lr,(br,br,br,0))

    h,w = frame_lr_shine.shape[:2]
    cv2.circle(frame_lr_shine, (int(w/2),int(h/2)), 2, (0,255,255),-1)

    frame_hsv = cv2.cvtColor(frame_lr_shine,cv2.COLOR_BGR2HSV) # BGR -> HSV
    lower_red = np.array([170,50,50]) #example value
    upper_red = np.array([180,255,255]) #example value
    mask_red = cv2.inRange(frame_hsv, lower_red, upper_red)
    # -> mask_red : binary image (using inRange function)

    # frame_red = cv2.bitwise_and(frame_lr_shine,frame_lr_shine,mask = mask_red)
    # -> 'and'calculation between original frame and mask

    red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(red_contours) > 0:
        print('number of contour : {}'.format(len(red_contours)))
        cmax = max(red_contours,key=cv2.contourArea)
        cv2.drawContours(frame_lr_shine, red_contours, -1, (0,255,255), 1 )
        M = cv2.moments(cmax)

        if M['m00']>0:
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(frame_lr_shine, (cX, cY), 2, (0, 255, 255), -1)
            print('center_of_red : ({x},{y})'.format(x=cX,y=cY))


    cv2.imshow('frame_lr', frame_lr_shine) #original frame + contours + centroid
    #cv2.imshow('frame_red', frame_red)

    if cv2.waitKey(10) == 27:
        break

cap.release()
cv2.destroyAllWindows()