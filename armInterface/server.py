import socket
from common import *

def main():
    listensocket = socket.socket()
    port = 8000

    listensocket.bind(('localhost', port))
    listensocket.listen(1)

    print(f"started on port {port}")

    (clientsocket, address ) = listensocket.accept()
    print(f"connected to {address}")

    terminated = False

    while not terminated:
        x = clientsocket.recv(1024).decode()
        print(x)

        terminated = term_stat(x)

if __name__ == "__main__":
    main()