# URC Socket Test Server
# Last Updated March 6th 2021 1:26 PM EST
# Created By: Cameron Robinson
# Last Update By: Cameron Robinson

import socket

listensocket = socket.socket()
port = 8000 # arbitrary, must match client ~CR
maxConnections = 999 #arbitrary large number, tbh IDK what it does but the internet says we need it
                     # I imagine that it is the amount of devices allowed to connect and could likely be much lower
                     # and maybe should be to reduce interference. May update later or in real version ~CR

IP = socket.gethostname() # This will report server IP for matching client ~CR

listensocket.bind(('',port)) #empty string could be client name but thats unnessicary, unsure of benefit so I avoided ~CR

listensocket.listen(maxConnections)
print("Server started at "+ IP + " on port "+ str(port)) #Start message. Reports IP for initilizing client ~CR

(clientsocket, address ) = listensocket.accept()
print("New connection made!")

running = True #no terminate message so you have to turn off. code easily modified to shut off on a certian message ~CR

while running:
    message = clientsocket.recv(1024).decode()
    print(message)

