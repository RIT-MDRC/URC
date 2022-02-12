import socket
import json

terminated = False

def execute_message(message):
    command = message.get('command')
    info    = message.get('info')

    match command:
        case 'terminate':
            print('Terminating...')
            global terminated; terminated = True
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
    listensocket = socket.socket()
    port = 8000
    listensocket.bind(('localhost', port))
    listensocket.listen(1)
    (clientsocket, address ) = listensocket.accept()

    print(f"started on port {port}")
    print(f"connected to {address}")
    while not terminated:
        x = clientsocket.recv(1024).decode()
        execute_message(json.loads(x))

if __name__ == "__main__":
    main()