import socket

s = socket.socket()
port = 8000 

s.connect(('localhost', port))

terminated = False

while not terminated:
    x = input("Enter message: ")
    s.send(x.encode())