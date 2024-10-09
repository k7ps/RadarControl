import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

import pygame

# compile & run
# os.system("cd build && make")
# os.system("./build/RadarControl")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

screen_width, screen_height = 900, 600
big_radar_radius = 400
main_radar_radius = 200
radar_angle_view = np.deg2rad(60)
cur_angle = np.deg2rad(30)
radar_position = (screen_width/2, screen_height - 5)

pygame.init()
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption('RadarControl')

def draw_sector(screen, col, pos, rad, angle_pos, angle_view, width=1):
    start_angle = angle_pos - angle_view/2
    end_angle = angle_pos + angle_view/2

    pygame.draw.arc(screen, col, 
        (pos[0] - rad, pos[1] - rad, 2 * rad, 2 * rad), 
        start_angle, end_angle, width=width
    )
    pygame.draw.line(screen, col, pos, (pos[0] + np.cos(start_angle) * rad, pos[1] - np.sin(start_angle) * rad), width=width)
    pygame.draw.line(screen, col, pos, (pos[0] + np.cos(end_angle) * rad, pos[1] - np.sin(end_angle) * rad), width=width)

start_angle = np.deg2rad(30)
end_angle = np.deg2rad(150)
step = np.deg2rad(0.5)
go_up = True

clock = pygame.time.Clock()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(WHITE)

    pygame.draw.circle(screen, RED, radar_position, 4)
    pygame.draw.circle(screen, BLACK, radar_position, big_radar_radius, width=1)

    if cur_angle <= start_angle:
        cur_angle += step
        go_up = True
    elif cur_angle >= end_angle:
        cur_angle -= step
        go_up = False
    elif go_up:
        cur_angle += step
    else:
        cur_angle -= step

    draw_sector(screen, BLACK, radar_position, main_radar_radius, cur_angle, radar_angle_view, 1)

    pygame.display.flip()

    dt = clock.tick(60)
    step = np.deg2rad(20) * dt / 1000
    
pygame.quit()

