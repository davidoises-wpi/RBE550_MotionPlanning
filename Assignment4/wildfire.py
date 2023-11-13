from pathlib import Path
import sys
import pygame
import environment
from vehicles import SimpleVehicleSprite
import wumpus as wp
import firetruck as ft
import threading

import time

FPS = 60

def render_all(window, clock, vehicles, bushes, wumpus_explored_states, wumpus_path, firetruck_explored_states, firetruck_path, time):
    window.fill(environment.WHITE)

    for bush in bushes:
        bush.render(window)

    for state in wumpus_explored_states:
        window.fill((0, 0, 0), ((state.x, state.y), (2, 2)))

    for point in wumpus_path:
        window.fill((255, 0, 0), ((point.x,point.y), (2, 2)))

    for state in firetruck_explored_states:
        window.fill((128, 0, 128), ((state.x, state.y), (2, 2)))

    for point in firetruck_path:
        window.fill((0, 0, 255), ((point.x,point.y), (2, 2)))

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

    project_root = Path(sys.path[0])

    pygame.init()

    screen = pygame.display.set_mode((environment.SCREEN_WIDTH_PIXELS, environment.SCREEN_HEIGHT_PIXELS))
    pygame.display.set_caption("Wildfire")

    clock = pygame.time.Clock()

    # Wumpus initialization
    wumpus_size = (5*environment.METERS_TO_PIXELS, 3.5*environment.METERS_TO_PIXELS)
    # wumpus = SimpleVehicleSprite(str(project_root) + "/assets/wumpus.png", 0, (0, 0), wumpus_size)
    wumpus = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 0, (100, 100), wumpus_size)
    wp.initialize_wumpus_initial_state(wumpus)

    firetruck_size = (10*environment.METERS_TO_PIXELS, 7*environment.METERS_TO_PIXELS)
    firetruck = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 180, (100, 100), firetruck_size)
    ft.initialize_firetruck_initial_state(firetruck, wumpus)

    environment.populate_map(0.1)

    firetruck_search_thread = threading.Thread(target=ft.firetruck_main, args=(firetruck,))
    firetruck_search_thread.daemon = True
    firetruck_search_thread.start()

    wumpus_search_thread = threading.Thread(target=wp.wumpus_main, args=(wumpus,))
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

        elapsed_time_real = pygame.time.get_ticks() - start_time_real
        elapsed_time_simulation = elapsed_time_real*environment.REAL_TO_SIMULATION_SECS_PER_MILLIS

        environment.update_environment(None, wumpus, elapsed_time_simulation)

        firetruck_path = []
        if ft.is_firetruck_path_ready():
            firetruck_path = ft.get_firetruck_path()
        firetruck_states = ft.get_firetruck_explored_states()

        wumpus_path = []
        if wp.is_wumpus_path_ready():
            wumpus_path = wp.get_wumpus_path()
        wumpus_states = wp.get_wumpus_explored_states()

        render_all(screen, clock, [wumpus, firetruck], environment.bushes, wumpus_states, wumpus_path, firetruck_states, firetruck_path, round(elapsed_time_simulation))

        if elapsed_time_simulation >= 3600.0:
            run = False

    run = True
    while run:
        wp.stop_playing = True
        ft.stop_playing = True

        intact_bushes = 0
        burned_bushes = 0
        extingguished_bushes = 0
        for bush in environment.bushes:
            if bush.state == environment.Bush.EXTINGUISHED_STATE:
                extingguished_bushes += 1
            elif bush.state == environment.Bush.BURNED_STATE:
                burned_bushes += 1
            elif bush.state == environment.Bush.NORMAL_STATE:
                intact_bushes += 1
        # Handle user events
        for event in pygame.event.get():
            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

    print("Total number of bushes", len(environment.bushes))
    print("Intact bushes", intact_bushes)
    print("Burned bushes", burned_bushes)
    print("Extinguished bushes", extingguished_bushes)

    # Finish execution
    pygame.quit()

if __name__ == "__main__":
    main()
