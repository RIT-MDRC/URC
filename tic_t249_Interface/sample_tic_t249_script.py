#################################################################
# Filename: simple_tic_t249.py                                  #
# Purpose: simple script to demonstrate use of tict249 library  #
# Authoer: Will Wright                                          #
# Date Created: 1/30/2022                                       #
# Date last Modified: 1/30/2022 by Will Wright                  #
#                                                               #
#################################################################
from smbus2 import SMBus, i2c_msg
import tic_t249_lib as tic_lib

#Open a handle to "/dev/i2c-1", representing the I2C bus.
bus = SMBus(1)
 
#Select the I2C address of the Tic (the device number).
address = 14

tic = tic_lib.TicI2C(bus, address)
 
tic.exit_safe_start() 

tic.energize()

tic.set_position(-90)

tic.set_position(90)

tic.set_position(0)

tic.deenergize()




