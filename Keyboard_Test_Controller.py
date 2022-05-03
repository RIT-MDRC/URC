import serial
import keyboard

# Change the port to whatever your computer sees (can check in the Arduino IDE)
teensy = serial.Serial(port='COM11',baudrate=115200, timeout=0.1)

while True:
    # Move Joint 1 with "1" and "2" keys
    if keyboard.is_pressed("1"):
        teensy.write(bytes("M1 80",'utf-8'))
    elif keyboard.is_pressed("2"):
        teensy.write(bytes("M1 -80",'utf-8'))
    else:
        teensy.write(bytes("M1 0",'utf-8'))
    
    # Move Joint 2 with "3" and "4" keys
    if keyboard.is_pressed("3"):
        teensy.write(bytes("M3 100",'utf-8'))
    elif keyboard.is_pressed("4"):
        teensy.write(bytes("M3 -100",'utf-8'))
    else:
        teensy.write(bytes("M3 0",'utf-8'))
        
