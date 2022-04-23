import pygame

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


pygame.init()

# screen = pygame.display.set_mode((500, 700))

# pygame.display.set_caption("My Game")

done = False

clock = pygame.time.Clock()

pygame.joystick.init()

# textPrint = TextPrint()

jointNum = 1
moveAmount = 0

lastHatValue = 0

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True


#     screen.fill(WHITE)
#     textPrint.reset()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

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
    elif hat[1] == 0:
        lastHatValue = 0
        
    gripperState = joystick.get_button(0)
    print(gripperState)
#     textPrint.unindent()
    
#     textPrint.tprint(screen, "M{} {}".format(jointNum, moveAmount))
    print("M{} {}".format(jointNum, moveAmount))

#     pygame.display.flip()

    clock.tick(20)

pygame.quit()
