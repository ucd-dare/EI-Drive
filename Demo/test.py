import pygame
import pygame.gfxdraw
import math


class Gauge:
    def __init__(self, screen, FONT, x_cord, y_cord, thickness, radius, circle_colour, glow=True):
        self.screen = screen
        self.Font = FONT
        self.x_cord = x_cord
        self.y_cord = y_cord
        self.thickness = thickness
        self.radius = radius
        self.circle_colour = circle_colour
        self.glow = glow

    def draw(self, percent):
        fill_angle = int(percent*270/100)
        per=percent
        if percent > 100:
            percent = 100
        if per <=40:
            per=0
        if per > 100:
            per = 100
        ac = [int(255-per*255/100),int(per*255/100),int(0), 255]
        for indexi in range(len(ac)):
            if ac[indexi] < 0:
                ac[indexi] = 0
            if ac[indexi] > 255:
                ac[indexi] = 255
        # print(ac)

        pertext = self.Font.render(str(percent) + "%", True, ac)
        pertext_rect = pertext.get_rect(center=(int(self.x_cord), int(self.y_cord)))
        self.screen.blit(pertext, pertext_rect)

        for i in range(0, self.thickness):

            pygame.gfxdraw.arc(screen, int(self.x_cord), int(self.y_cord), self.radius - i, -225, 270 - 225, self.circle_colour)
            if percent >4:
                pygame.gfxdraw.arc(screen, int(self.x_cord), int(self.y_cord), self.radius - i, -225, fill_angle - 225-8, ac)

        if percent < 4:
            return

        if self.glow:
            for i in range(0,15):
                ac [3] = int(150 - i*10)
                pygame.gfxdraw.arc(screen, int(self.x_cord), int(self.y_cord), self.radius + i, -225, fill_angle - 225-8, ac)

            for i in range(0,15):
                ac [3] = int(150 - i*10)
                pygame.gfxdraw.arc(screen, int(self.x_cord), int(self.y_cord), self.radius -self.thickness - i, -225, fill_angle - 225-8, ac)

            angle_r = math.radians(fill_angle-225-8)
            lx,ly = int((self.radius-self.thickness/2)*math.cos(angle_r)), int( (self.radius-self.thickness/2)*math.sin(angle_r))
            ac[3] = 255
            lx = int(lx+self.x_cord)
            ly = int(ly + self.y_cord)

            pygame.draw.circle(self.screen,ac,(lx,ly),int(self.thickness/2),0)


            for i in range(0,10):
                ac [3] = int(150 - i*15)
                pygame.gfxdraw.arc(screen, int(lx), int(ly), (self.thickness//2)+i , fill_angle -225-10, fill_angle - 225-180-10, ac)



if __name__ == '__main__':
    bg_c = (56, 56, 56)
    circle_c = (55, 77, 91)

    pygame.init()
    width, height = (640, 480)
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((width, height))


    pygame.display.set_caption('Wahaj Gauge Pygame')


    fps = 10
    FONT = pygame.font.SysFont('Franklin Gothic Heavy', 100)

    my_gauge = Gauge(
        screen=screen,
        FONT=FONT,
        x_cord=width / 2,
        y_cord=height / 2,
        thickness=50,
        radius=200,
        circle_colour=circle_c,
        glow=False)

    percentage = 0
    while True:

        # FOR SHOWING CHANGE IN GAUGE
        percentage+=1
        if percentage > 100:
            percentage = 0

        screen.fill(bg_c)
        my_gauge.draw(percent=percentage)
        pygame.display.update()
        clock.tick(fps)
