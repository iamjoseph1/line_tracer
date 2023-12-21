import os
import numpy as np
import cv2
import torch
from Motor_Control import *
#from yolo_test import *

################################
def load_model():
    print(f"[INFO] Loading model... ")
    ## loading the custom trained model
    # model =  torch.hub.load('ultralytics/yolov5', 'custom', path='last.pt',force_reload=True) ## if you want to download the git repo and then run the detection
    model =  torch.hub.load('.', 'custom', source ='local', 
                            path='best_v2.pt',force_reload=True)    #best_v2.pt should be in 'yolov5' folder
    classes = model.names ### class names in string
    
    return model, classes

def detect_(frame, model):
    frame = [frame]
    print(f"[INFO] Detecting. . . ")
    results = model(frame)
    # results.show()
    # print(results.xyxyn[0])
    #print(results.xyxyn[0][:, -1])      # print labels
    #print(results.xyxyn[0][:, :-1])     # print cordinates

    labels, cordinates = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

    return labels, cordinates

###############################


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
DEFAULT_VELOCITY = 20 # Need tuning

# load model
model, classes = load_model()

# Open camera
cap = cv2.VideoCapture(CAMERANAME, cv2.CAP_V4L) # Vedio4Linux
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

# PID Gain -> need tuning
Kp = 0.3
Ki = 0
Kd = 0

error_b = 0
error_i = 0

cX = 0
cY = 0

timer_stop = 0
timer_slow = 0
count_slow = 0


stop_count = 0


fpslimit = 1.5
start_time = time.time()
# Detect line & objects
while True:

    # Frame setting
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))    
        
    frame = cv2.flip(frame,1)
    #frame = frame[80:160,0:120]
    h,w = frame.shape[:2]
    #cv2.circle(frame, (int(w/2),int(h/2)), 2, (0,255,255),-1)

    # Detect line
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame_blur = cv2.GaussianBlur(frame_gray,(5,5),0)
    ret, thresh1 = cv2.threshold(frame_blur,100,255,cv2.THRESH_BINARY_INV)

    now_time = time.time()
    if (now_time - start_time) >= fpslimit:

        contours, _ = cv2.findContours(thresh1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
        # Control Motors
        if len(contours) > 0 and stop_count!=1:
            yolo_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            cmax = max(contours,key=cv2.contourArea)
            cv2.drawContours(frame, contours, -1, (0,255,255), 1 )
            M = cv2.moments(cmax)

            if M['m00']>0:
                cX = int(M['m10']/M['m00'])
                cY = int(M['m01']/M['m00'])
                cv2.circle(frame, (cX, cY), 2, (0, 255, 255), -1)

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

                if time.time()-timer_slow < 3 and count_slow > 0:                   # To prevent it from going slowly for the first 3 seconds 
                    DXL.Dual_MotorController(int(left_vel/2), int(right_vel/2))
                else:
                    DXL.Dual_MotorController(left_vel, right_vel)

                
                # Detect objects
                labels, cord = detect_(yolo_frame, model = model)

                # Detect only one object at a time (highest confidence score)
                if len(labels):
                    confidence = cord[:, -1]
                    array = confidence.numpy()
                    max_idx = np.argmax(array)
                    row = cord[max_idx]

                    if row[4] >= 0.75: ### threshold
                        x1, y1, x2, y2 = int(row[0]*w), int(row[1]*h), int(row[2]*w), int(row[3]*h) #coordinates
                        text_d = classes[int(labels[max_idx])]
                        
                        #cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0,255), 2)
                        #cv2.rectangle(frame, (x1, y1-20), (x2, y1), (0, 0,255), -1)
                        #cv2.putText(frame, text_d + f" {round(float(row[4]),2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,255,255), 2)

                        if text_d == 'left':
                            DXL.Dual_MotorController(-20, -20)
                            time.sleep(1)

                        elif text_d == 'right':
                            DXL.Dual_MotorController(20, 20)
                            time.sleep(1)

                        elif text_d == 'stop' and stop_count == 0: #and time.time()-timer_stop >= 3:      # to escape from 'stop' sign
                            stop_count +=1
                            
                            #if stop_count ==0:
                                #stop_count +=1
                                #DXL.Dual_MotorController(0, 0)
                                #time.sleep(1)
                                #timer_stop = time.time()
                            #elif stop_count != 0:
                                #continue

                        elif text_d == 'uturn':
                            DXL.Dual_MotorController(-20, -20)
                            time.sleep(3)

                        elif text_d == 'slow' and time.time()-timer_slow >= 3:
                            print('slowwwwww')
                            DXL.Dual_MotorController(int(left_vel/2), int(right_vel/2))
                            timer_slow = time.time()
                            count_slow += 1 # To prevent it from going slowly for the first 3 seconds



                if stop_count ==1:
                    print("STOPPPPPPPPP")
                    DXL.Dual_MotorController(0,0)
                    time.sleep(2)
                    stop_count = 2

            else :
                DXL.Dual_MotorController(0,0)
                
        start_time = time.time()
    #cv2.imshow('frame', frame)                  # orignal frame(fliped) with contours + centroid + objects
    cv2.imshow('frame_thresh', thresh1)         # binary frame with line

    if cv2.waitKey(10) == ord('q'):
        DXL.Dual_MotorController(0,0)
        break

# Close camera
cap.release()
cv2.destroyAllWindows()

# Disable torque on Motor & Close Port
DXL.Unconnect_Motor()
