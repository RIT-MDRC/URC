# URC Socket Test Client
# Last Updated March 6th 2021 1:20 PM EST
# Created By: Cameron Robinson
# Last Update By: Cameron Robinson

import socket
s = socket.socket()

hostname = 'Camerons-Mac-mini.local' #replace with hostname given by running server ~CR
port = 8000 # arbitrary, must match server ~CR

s.connect((hostname,port))

while True:
    x = input("Enter message: ") 
    s.send(x.encode())
    
    