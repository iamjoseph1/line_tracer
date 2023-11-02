import os
import numpy as np
import cv2
import time

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_DRIVE_MODE             = 10               # Control table address is different in Dynamixel model
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_PWM               = 100

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [1,2]             # Dynamixel ID
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DRIVE_NORMAL                = 0                 #positive value : CCW
DRIVE_REVERSE               = 1                 #positive value : CW
PWM_CONTROL_MODE            = 16                # Value for PWM control mode (operating mode)
PWM_LIMIT                   = 885

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Set operating mode to velocity control mode & Enable Torque
for i in range(0,1):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d Operating mode changed to PWM control mode." % DXL_ID[i])
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID[i])

    
#define motor control function
#뒤쪽에서 보았을 때 왼쪽모터 : DXL_ID1, 오른쪽모터 : DXL_ID2
#두 모터 다 DRIVE_MODE : NORMAL_MODE(반시계방향(CCW)이 양수값, 시계방향(CW)이 음수값)으로 설정되어 있음
def goahead(num):
    error_i = 0 #방향을 잡으면 에러 누적값을 초기화
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[0], ADDR_GOAL_PWM, num)
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[1], ADDR_GOAL_PWM, -num)

def turnleft(num):
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[0], ADDR_GOAL_PWM, -num)
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[1], ADDR_GOAL_PWM, -num)

def turnright(num):
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[0], ADDR_GOAL_PWM, num)
    packetHandler.write2ByteTxRx(port_handler, DXL_ID[1], ADDR_GOAL_PWM, num)

#open camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print('Camera Open Failed..!')
    exit()

#set start time for timer
#start_time = time.time()

#PID Gain -> need tuning
Kp = 1.0
Ki = 0.5
Kd = 1.0

error_b = 0
error_i = 0

cX = 0
cY = 0

#detect line
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

    #timer for reset error_i
    #if time.time()-start_time>=0.1: #노트북 웹켐 fps : 30이므로, 약 3주기마다 에러 누적값 초기화
    #    error_i = 0
    #    start_time = time.time()

    #PID control for motor_rpm
    if cX > 0:
        error_p = w/2-cX
        error_i += error_p
        error_d = error_b-error_p
        error_control = Kp*error_p + Ki*error_i + Kd*error_d
        error_b = error_p
        print('error : %d       error_control : %d' %(error_p, error_control))


        #call motor control function
        if error_p>5:
            print('turn left & motor_PWM : %d' % min(abs(error_control),PWM_LIMIT))
            turnleft(int(min(abs(error_control),PWM_LIMIT)))
        elif abs(error_p) <= 5:
            print('go ahead & motor_PWM : 440')
            goahead(440) #half of PWM_LIMIT
        else:
            print('turn right & motor_PWM : %d' % min(abs(error_control),PWM_LIMIT))
            turnright(int(min(abs(error_control),PWM_LIMIT)))

    if cv2.waitKey(10) == 27:
        break

# Close camera
cap.release()
cv2.destroyAllWindows()

# Disable Dynamixel Torque
for i in range(0,1):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disconnected" % DXL_ID[i])
        
# Close port
portHandler.closePort()