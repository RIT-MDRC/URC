#################################################################
# Filename: tic_t249_lib.py                                     #
# Purpose: tic t249 motor control library for MDRC robotic arm  #
# Author: Will Wright                                          #
# Date Created: 1/25/2022                                       #
# Date last Modified: 1/30/2022 by Will Wright                  #
#                                                               #
################################################################# 
from smbus2 import SMBus, i2c_msg
import time
import math

 # class for i2c communication with a Tic T249 motor controller
class TicI2C(object):
    
    # bus: int : the i2c bus interface on the raspberry pi (using smbus) 
    # address: int : the i2c address of the motor controller
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
 
    # Sends the "Exit safe start" command, this disables "Exit safe start" error.
    def exit_safe_start(self):
        command = [0x83]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
 
    # Sets the target position.
    # target: int : position to set the motor in microsteps (−0x8000 0000 to +0x7FFF FFFF) 
    def set_target_position(self, target):
        command = [0xE0,
          target >> 0 & 0xFF,
          target >> 8 & 0xFF,
          target >> 16 & 0xFF,
          target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
    
    # Sets the velocity of the motor, however..... it doesn't work as expected :(
    # target: int : position to set the motor in microsteps/10,000s (−500,000,000 to +500,000,000) 
    def set_target_velocity(self, target):
        command = [0xE3,
          target >> 0 & 0xFF,
          target >> 8 & 0xFF,
          target >> 16 & 0xFF,
          target >> 24 & 0xFF]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)

    # Energizes the motor
    def energize(self):
        command = [0x85]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
        time.sleep(0.5)
        print("Energizing motor...")
    
    # De-energizes the motor
    def deenergize(self):
        command = [0x86]
        write = i2c_msg.write(self.address, command)
        self.bus.i2c_rdwr(write)
        time.sleep(0.5)
        print("De-energizing motor...")
 
    # Gets one or more variables from the Tic.
    # offset: int : position to set the motor in microsteps/10,000s
    # target: int : position to set the motor in microsteps/10,000s
    def get_variables(self, offset, length):
        write = i2c_msg.write(self.address, [0xA1, offset])
        read = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write)
        self.bus.i2c_rdwr(read)
        return list(read)
 
    # Gets the "Current position" variable from the Tic.
    def get_current_position(self):
        b = self.get_variables(0x22, 4)
        position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
        if position >= (1 << 31):
            position -= (1 << 32)
        return position
 
    # function to turn the motor a set number of degrees, this is used in actual control of motor from kinematics
    # degrees: int : number of degrees to turn motor 
    def set_position(self, degrees):
        # finds correct amount of tics to move motor based on desired degrees
        adjusted_degrees = int(degrees * 2.855555555)
        print("Setting target position to {}.".format(degrees))
        
        # sets position in degrees
        current_pos = int(math.ceil(self.get_current_position()/2.855555555))
        print("current position: "+str(current_pos))
        print("future position: "+str(degrees))
        
        # finds correct amount of time to wait for turn (slght margin of error, actual constatn is 0.01431)
        wait_time = 0.02*abs(degrees-current_pos)
        print("adjusted_degrees "+str(adjusted_degrees))
        self.set_target_position(adjusted_degrees)
        # waits correct amount of time
        print("wait time: "+str(wait_time)+"\n")
        time.sleep(wait_time)
