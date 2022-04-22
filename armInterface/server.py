import numpy as np
import socket
import json
import serial
import time

# move this to a different place
class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Server:
    def __init__(self, listen_socket = socket.socket(), port = 8000, address = 'urc.student.rit.edu'):
        self.terminated     = False
        self.listen_socket  = listen_socket
        self.port           = port
        self.address        = address
        self.vec            = Vector(0, 0, 0)

    def start_server(self):
        self.listen_socket.bind((self.address, self.port))
        self.listen_socket.listen(1)
        (clientsocket, client_address) = self.listen_socket.accept()

        print(f"started on port {self.port}")
        print(f"connected to {client_address}")
        while not self.terminated:
            x = clientsocket.recv(1024).decode()
            self.execute_message(json.loads(x))


    def execute_message(self, message):
        command = message.get('command')
        info    = message.get('info')
        send_to_board(command)
        
"""
        if command == 'terminate':
            print('Terminating...')
            self.terminated = True
        elif command == 'x_offset':
            print(f'Move {info} x')
        elif command == 'y_offset':
            print(f'Move {info} y')
        elif command == 'z_offset':
            print(f'Move {info} z')
        elif command == 'rotate_base':
            degrees = info['degrees']
            direction = info['direction']
            print(f'rotate base {degrees} degrees {direction}')
        elif command == 'rotate_end_effector':
            degrees = info['degrees']
            direction = info['direction']
            print(f'rotate end effector {degrees} degrees {direction}')

        else:
            print('Invalid json  :(')
"""

def send_to_board(message):
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.reset_input_buffer()
    print(message)
#    while True:
    ser.write(bytes(message, 'utf-8'))
#        line = ser.readline().decode('utf-8').rstrip()
#        print(line)
#        time.sleep(1)

def main():
    server = Server()
    server.start_server()


if __name__ == "__main__":
    main()
