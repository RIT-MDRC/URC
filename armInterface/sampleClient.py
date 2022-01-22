import socket
from common import *

s = socket.socket()
port = 8000 

s.connect(('localhost' ,port))

terminated = False

while not terminated:
    x = input("Enter message: ")
    s.send(x.encode())

    terminated = term_stat(x)
