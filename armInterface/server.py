import numpy as np
import socket
import json

# move this to a different place
class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Server:
    def __init__(self, listen_socket = socket.socket(), port = 8000, address = 'localhost'):
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

        match command:
            case 'terminate':
                print('Terminating...')
                self.terminated = True
            case 'x_offset':
                print(f'Move {info} x')
            case 'y_offset':
                print(f'Move {info} y')
            case 'z_offset':
                print(f'Move {info} z')
            case 'rotate_base':
                degrees = info['degrees']
                direction = info['direction']
                print(f'rotate base {degrees} degrees {direction}')
            case 'rotate_end_effector':
                degrees = info['degrees']
                direction = info['direction']
                print(f'rotate end effector {degrees} degrees {direction}')

            case _:
                print('Invalid json  :(')

def main():
    server = Server()
    server.start_server()


if __name__ == "__main__":
    main()