import socket

class Client:
    def __init__(self, send_socket = socket.socket(), port = 8000, address = 'urc.student.rit.edu'):
        self.terminated     = False
        self.send_socket    = send_socket
        self.port           = port
        self.address        = address

    def client_start(self):
        self.send_socket.connect((self.address, self.port))

        while not self.terminated:
            message = input("Enter message: ")
            self.send_socket.send(message.encode())

def main():
    client = Client()
    client.client_start()

if __name__ == "__main__":
    main()
