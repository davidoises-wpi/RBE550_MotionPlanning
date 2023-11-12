from pathlib import Path
import sys
import pygame
import environment
from vehicles import SimpleVehicleSprite
from wumpus import wumpus_main, get_wumpus_path, is_wumpus_path_ready, initialize_wumpus_initial_state, get_wumpus_explored_states
import threading

from math import cos, sin, radians, degrees

FPS = 60

def render_all(window, clock, vehicles, bushes, wumpus_explored_states, time):
    window.fill(environment.WHITE)

    for bush in bushes:
        bush.render(window)

    for state in wumpus_explored_states:
        window.fill((0, 0, 0), ((state.x, state.y), (2, 2)))

    for vehicle in vehicles:
        # vehicle.set_position(point[0], point[1])
        # vehicle.set_orientation(degrees(point[2]))
        vehicle.update()
        vehicle.render(window)

    font = pygame.font.Font('freesansbold.ttf', 32)
    text = font.render(str(time),True, (0,0,0),None)
    text_rect = text.get_rect()
    text_rect.topright = (environment.SCREEN_WIDTH_PIXELS, 0)

    window.blit(text, text_rect)

    pygame.display.update()

    clock.tick(FPS)

def main():

    steps = 10
    forward = False
    reverse = False
    rotate_left = False
    rotate_right = False

    project_root = Path(sys.path[0])

    pygame.init()

    screen = pygame.display.set_mode((environment.SCREEN_WIDTH_PIXELS, environment.SCREEN_HEIGHT_PIXELS))
    pygame.display.set_caption("Wildfire")

    clock = pygame.time.Clock()

    car = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 180, (100, 100))

    # Wumpus initialization
    # wumpus_size = (2*environment.METERS_TO_PIXELS, 2*environment.METERS_TO_PIXELS)
    wumpus_size = (10*environment.METERS_TO_PIXELS, 10*environment.METERS_TO_PIXELS)
    wumpus = SimpleVehicleSprite(str(project_root) + "/assets/wumpus.png", 0, (0, 0), wumpus_size)
    initialize_wumpus_initial_state(wumpus)

    environment.populate_map(0.1)

    wumpus_search_thread = threading.Thread(target=wumpus_main, args=(wumpus,))
    wumpus_search_thread.daemon = True
    wumpus_search_thread.start()

    start_time_real = pygame.time.get_ticks()

    """ Main loop with search and visualization """
    run = True
    while run:

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

        car_x_step = steps*cos(radians(car.orientation))
        car_y_step = -steps*sin(radians(car.orientation))
        car_angle_step = 1

        if forward or reverse:
            if reverse:
                car_x_step *= -1
                car_y_step *= -1
            car.set_position(car.x + car_x_step, car.y + car_y_step)

        if rotate_left or rotate_right:
            if rotate_right:
                car_angle_step *= -1
            car.set_orientation(car.orientation + car_angle_step)

        elapsed_time_real = pygame.time.get_ticks() - start_time_real
        elapsed_time_simulation = elapsed_time_real*environment.REAL_TO_SIMULATION_SECS_PER_MILLIS

        environment.update_environment(None, wumpus, elapsed_time_simulation)

        # This is just to render the path
        # if is_wumpus_path_ready():
        #     print("ready")
        #     path = get_wumpus_path()
        #     print(path)
        wumpus_states = get_wumpus_explored_states()

        render_all(screen, clock, [wumpus, car], environment.bushes, wumpus_states, round(elapsed_time_simulation))

        if elapsed_time_simulation >= 3600.0:
            run = False

    run = True
    while run:
        # Handle user events
        for event in pygame.event.get():
            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

    # Finish execution
    pygame.quit()

if __name__ == "__main__":
    main()
