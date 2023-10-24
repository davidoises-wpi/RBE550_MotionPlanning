from pathlib import Path
import sys
import pygame
from math import radians, cos, sin

from vehicles import SimpleVehicleSprite

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

def main():

    project_root = Path(sys.path[0])

    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Motion Planning With Kinematic Constraints")

    clock = pygame.time.Clock()
    fps = 30

    car = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 180, (200, 200))
    delivery_robot = SimpleVehicleSprite(str(project_root) + "/assets/skid_mobile_robot.png", 90, (300, 300))

    #create bullet and mask
    bullet = pygame.Surface((10, 10))
    bullet.fill(RED)
    bullet_mask = pygame.mask.from_surface(bullet)
    bullet_rect = bullet.get_rect()
    bullet_rect.center = (300,300)


    steps = 10
    forward = False
    reverse = False
    rotate_left = False
    rotate_right = False

    run = True
    while run:

        #get mouse coordinates
        # pos = pygame.mouse.get_pos()
        # print(pos, end=" : ")
        # print(vehicle_rect.center, end=" : ")
        # print(vehicle_angle, end=" : ")

        # bullet_rect.center = pos
        col = GREEN

        screen.fill(WHITE)

        # # First check rect overlap
        # if vehicle_rect.colliderect(bullet_rect):
        #     # check mask overlap
        #     if vehicle_mask.overlap(bullet_mask, (bullet_rect[0]-vehicle_rect[0], bullet_rect[1]-vehicle_rect[1])):
        #         col = RED

        bullet.fill(col)
        screen.blit(bullet,bullet_rect)

        # Handle user events
        for event in pygame.event.get():

            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT or event.key == ord('a'):
                    rotate_left = True
                if event.key == pygame.K_RIGHT or event.key == ord('d'):
                    rotate_right = True
                if event.key == pygame.K_UP or event.key == ord('w'):
                    forward = True
                if event.key == pygame.K_DOWN or event.key == ord('s'):
                    reverse = True

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == ord('a'):
                    rotate_left = False
                if event.key == pygame.K_RIGHT or event.key == ord('d'):
                    rotate_right = False
                if event.key == pygame.K_UP or event.key == ord('w'):
                    forward = False
                if event.key == pygame.K_DOWN or event.key == ord('s'):
                    reverse = False

        delivery_robot_x_step = steps*cos(radians(delivery_robot.orientation))
        delivery_robot_y_step = -steps*sin(radians(delivery_robot.orientation))
        delivery_robot_angle_step = 2

        car_x_step = steps*cos(radians(car.orientation))
        car_y_step = -steps*sin(radians(car.orientation))
        car_angle_step = 1

        if forward or reverse:
            if reverse:
                car_x_step *= -1
                car_y_step *= -1

                delivery_robot_x_step *= -1
                delivery_robot_y_step *= -1
            car.set_position(car.x + car_x_step, car.y + car_y_step)
            delivery_robot.set_position(delivery_robot.x + delivery_robot_x_step, delivery_robot.y + delivery_robot_y_step)

        if rotate_left or rotate_right:
            if rotate_right:
                delivery_robot_angle_step *= -1
                car_angle_step *= -1
            car.set_orientation(car.orientation + car_angle_step)
            delivery_robot.set_orientation(delivery_robot.orientation + delivery_robot_angle_step)

        car.render(screen)
        delivery_robot.render(screen)

        pygame.display.update()
        clock.tick(fps)

    # Finish execution
    pygame.quit()

if __name__ == "__main__":
    main()
