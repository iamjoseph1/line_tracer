import os
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

from dynamixel_sdk import * # Uses Dynamixel SDK library
# from DynamixelSDK.python.src.dynamixel_sdk import *   # import from path

import time

class Motor_Controller:
    def __init__(self):
        self.MY_DXL                      = 'X_SERIES'        # OUR DYNAMIXEL : xl430-w250-t
        self.BAUDRATE                    = 57600

        # self.DEVICENAME                  = "COM8"            # This is for Windows
        self.DEVICENAME                  = "/dev/ttyUSB0"    # Check your port name
        self.PROTOCOL_VERSION            = 2.0               # Use protocol 2.0 for dynamixel motors

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_VELOCITY          = 104               # Goal Velocity Address @ 104
        self.ADDR_PRESENT_VELOCITY       = 128               # Present Velocity Address @ 128

        self.RIGHT_ID                    = 0                 # Dynamixel#1 ID : 0
        self.LEFT_ID                     = 1                 # Dynamixel#2 ID : 1

        self.TARGET_VEL                  = 0                 # Target Velocity Value
        self.MAXIMUM_VELOCITY            = 1023              # Velocity MAX limit
        self.MINIMUM_VELOCITY            = -1023             # Velocity min limit

        self.ERROR_THRESHOLD             = 50                # Set Threshold for Velocity Error

        # Initialize port handler and packet handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port and set baudrate
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Enable torque on motors
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.RIGHT_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE) # Add closing parenthesis and store return values
        if dxl_comm_result != COMM_SUCCESS: # Check communication result
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0: # Check error code
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.LEFT_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE) # Add closing parenthesis and store return values
        if dxl_comm_result != COMM_SUCCESS: # Check communication result
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0: # Check error code
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def write_goal_velocity(self, id, velocity):
        try:
            self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_GOAL_VELOCITY, velocity)
        except Exception as e:
            print("Error while writing goal velocity:", e)

    def read_present_velocity(self, id):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, self.ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print("Failed to read present velocity. Error: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Error in reading present velocity: %s" % self.packetHandler.getRxPacketError(dxl_error))
        return dxl_present_velocity


    def MotorController(self, id, target_velocity):
        # Write goal velocity to motor @ id
        self.write_goal_velocity(id, target_velocity)

        # Wait until motors reach goal velocity
        while True:
            # Read present velocity from motors
            present_velocity = self.read_present_velocity(id)

            # Print current positions of the motors
            print("[ID:%03d] GoalVel:%03d PresentVel:%03d" \
                % (id, target_velocity, present_velocity))
            
            # use threshold value instead of DXL_MINIMUM_POSITION_VALUE_FOR_MOVING
            if abs(present_velocity - target_velocity) <= self.ERROR_THRESHOLD:
                break

    def Dual_MotorController(self, left_vel, right_vel):
        # Write goal velocity to motor @ id
        self.write_goal_velocity(0, right_vel)
        self.write_goal_velocity(1, left_vel)

        # # Wait until motors reach goal velocity
        # while True:
        #     # Read present velocity from motors
        #     R_present_velocity = self.read_present_velocity(0)
        #     L_present_velocity = self.read_present_velocity(1)

        #     # # Print current positions of the motors
        #     # print("[ID:%03d] GoalVel:%03d PresentVel:%03d" \
        #     #     % (id, target_velocity, present_velocity))
            
        #     # use threshold value instead of DXL_MINIMUM_POSITION_VALUE_FOR_MOVING
        #     if abs(R_present_velocity - right_vel) <= self.ERROR_THRESHOLD and abs(L_present_velocity - left_vel) <= self.ERROR_THRESHOLD:
        #         break

    def Unconnect_Motor(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.RIGHT_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE) # Add closing parenthesis and store return values
        if dxl_comm_result != COMM_SUCCESS: # Check communication result
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0: # Check error code
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("Dynamixel#%d has been successfully disconnected" % self.RIGHT_ID)

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.LEFT_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE) # Add closing parenthesis and store return values
        if dxl_comm_result != COMM_SUCCESS: # Check communication result
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0: # Check error code
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("Dynamixel#%d has been successfully disconnected" % self.LEFT_ID)

        self.portHandler.closePort()

if __name__=="__main__":

    RIGHT_ID                    = 0                 # Dynamixel#1 ID : 0
    LEFT_ID                     = 1                 # Dynamixel#1 ID : 1
    
    DXL = Motor_Controller()

    # DXL.MotorController(RIGHT_ID, 20)
    # time.sleep(3)
    DXL.MotorController(RIGHT_ID, 0)
    # time.sleep(3)

    # DXL.MotorController(LEFT_ID, 50)
    # time.sleep(3)
    DXL.MotorController(LEFT_ID, 0)
    time.sleep(3)