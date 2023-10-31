from pathlib import Path
import sys
import pygame
import environment
from vehicles import SimpleVehicleSprite

FPS = 60

def render_all(window, clock, vehicle, bushes):
    window.fill(environment.WHITE)

    for bush in bushes:
        bush.render(window)

    # vehicle.set_position(point[0], point[1])
    # vehicle.set_orientation(degrees(point[2]))
    vehicle.update()
    vehicle.render(window)

    pygame.display.update()

    clock.tick(FPS)

def main():

    project_root = Path(sys.path[0])

    pygame.init()

    screen = pygame.display.set_mode((environment.SCREEN_WIDTH_PIXELS, environment.SCREEN_HEIGHT_PIXELS))
    pygame.display.set_caption("Wildfire")

    clock = pygame.time.Clock()

    car = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 180, (100, 100))

    bushes = environment.populate_map(0.1)
    print(len(bushes))

    """ Main loop with search and visualization """
    run = True
    while run:

        # Handle user events
        for event in pygame.event.get():
            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

        render_all(screen, clock, car, bushes)

    # Finish execution
    pygame.quit()

if __name__ == "__main__":
    main()
