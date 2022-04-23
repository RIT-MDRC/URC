import socket
import pygame

pygame.init()
clock = pygame.time.Clock()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

class Client:
    def __init__(self, send_socket = socket.socket(), port = 8000, address = 'urc.student.rit.edu'):
        self.terminated     = False
        self.send_socket    = send_socket
        self.port           = port
        self.address        = address

    def client_start(self):
        self.send_socket.connect((self.address, self.port))
        jointNum = 1
        moveAmount = 0

        lastHatValue = 0
        
        gripperState = 0

        while not self.terminated:
            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.terminated = True

                try:
                    jid = joystick.get_instance_id()
                except AttributeError:
                    jid = joystick.get_id()

                leftTrigger = joystick.get_axis(2)
                rightTrigger = joystick.get_axis(5)
                if leftTrigger > 0 and rightTrigger > 0:
                    moveAmount = 0
                elif leftTrigger > 0:
                    moveAmount = -50
                elif rightTrigger > 0:
                    moveAmount = 50
                else:
                    moveAmount = 0

                hat = joystick.get_hat(0)
                if lastHatValue != hat[1] and hat[1] == 1 and jointNum != 3:
                    lastHatValue = 1
                    jointNum += 1
                elif lastHatValue != hat[1] and hat[1] == -1 and jointNum != 1:
                    lastHatValue = -1
                    jointNum -= 1
                if hat[1] == 0:
                    lastHatValue = 0
                    
                if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP: 
                    message2 = "\"command\"" + ":\"G{}\"".format(joystick.get_button(0))
                    message2 = '{' + message2 + '}'
                    self.send_socket.send(message2.encode())                
                    print(message2)
                elif event.type == pygame.JOYAXISMOTION:
                    message = "\"command\"" + ":\"M{} {}\"".format(jointNum, moveAmount)
                    message = '{' + message + '}'
                    self.send_socket.send(message.encode())
                    print(message)

            clock.tick(20)

def main():
    client = Client()
    client.client_start()

if __name__ == "__main__":
    main()
