from pathlib import Path
import sys
import pygame
from math import radians, cos, sin, degrees

from vehicles import SimpleVehicleSprite
from obstacles import Obstacle
from states import AckermannDriveState

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 650

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

FPS = 60

def reconstruct_path(state_chained_list):
    path = []
    path.append(state_chained_list.pop())
    index = 0
    while path[index].parent_state:
        path.append(path[index].parent_state)
        index += 1
    path.reverse()
    return path

def search_path(vehicle, obstacles, end_state):
    max_iterations = 10000
    dt = 0.05

    # Actual planning
    if search_path.open_states and search_path.iterations < max_iterations:

        # Search and remove the state with the lowest cost
        current_state = search_path.open_states[0]
        state_index = 0
        i = 0
        for state in search_path.open_states:
            if state.total_cost < current_state.total_cost:
                current_state = state
                state_index = i
            i += 1
        del search_path.open_states[state_index]

        # Add the current state to the closed list
        search_path.closed_states.append(current_state)

        if current_state.goal_check(end_state):

            path = reconstruct_path(search_path.closed_states)

            print("success")
            print("Finished reconstructing path")
            print("Number of iterations: ", search_path.iterations)
            print("Number of waypoints: ", len(path))
            print("Press any key to visualize the path")

            exit_code = 1
            return exit_code, path, search_path.closed_states
        else:
            search_path.iterations += 1
            neighbors = current_state.get_neighbors(vehicle, obstacles, dt, search_path.open_states, search_path.closed_states, end_state, (0, SCREEN_WIDTH), (0, SCREEN_HEIGHT))
            search_path.open_states += neighbors
            exit_code = 0
            return exit_code, [], search_path.closed_states
    else:
        print("Solution not found")
        exit_code = -1
        return exit_code, [], search_path.closed_states

def render_all(window, clock, vehicle, obstacles, explored_states, points_in_route):
    window.fill(WHITE)

    for obstacle in obstacles:
        obstacle.render(window)

    for state in explored_states:
        window.fill((0, 0, 0), ((state.x, state.y), (2, 2)))

    if points_in_route:

        point = points_in_route[-1]
        vehicle.set_position(point[0], point[1])
        vehicle.set_orientation(degrees(point[2]))
        vehicle.update()
        vehicle.render(window)

        for point in points_in_route:
            window.fill((0, 255, 0), ((point[0],point[1]), (2, 2)))

    pygame.display.update()

    clock.tick(FPS)

def main():

    """ This section is mostly initializing the map and objects in it """
    project_root = Path(sys.path[0])

    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Motion Planning With Kinematic Constraints")

    clock = pygame.time.Clock()

    obstacles = []
    obstacles.append(Obstacle((450,250), 400, 400, RED))
    obstacles.append(Obstacle((100,610), 200, 80, RED))
    obstacles.append(Obstacle((500,610), 200, 80, RED))
    obstacles.append(Obstacle((SCREEN_WIDTH/2,SCREEN_HEIGHT+1), SCREEN_WIDTH, 2, RED))
    obstacles.append(Obstacle((SCREEN_WIDTH/2,-1), SCREEN_WIDTH, 2, RED))

    car = SimpleVehicleSprite(str(project_root) + "/assets/tesla.png", 180, (100, 100))

    """ Declaring initial and final tates"""

    initial_state = AckermannDriveState(100, 100, radians(0), 0, 0, 0)

    # Multiple goals for testing
    # goal_state = AckermannDriveState(120, 300, radians(-90), 0, 0, 0)
    goal_state = AckermannDriveState(300, 600, radians(0), 0, 0, 0)
    # goal_state = AckermannDriveState(720, 220, radians(90), 0, 0, 0)

    """ Variables used for the search algorithm """
    search_path.open_states = [initial_state]
    search_path.closed_states = []
    search_path.iterations = 0

    """ Variables used for flow of the simulation"""
    path = []
    path_ready = 0
    stop = 0
    start_visualizing = 0

    """ Main loop with search and visualization """
    run = True
    while run:

        # Handle user events
        for event in pygame.event.get():
            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

            if path_ready and event.type == pygame.KEYDOWN:
                start_visualizing = 1

        """ A* search """
        if path_ready == 0 and not stop:
            path_ready, path, explored_states = search_path(car, obstacles, goal_state)

        """ Path visualization """
        points_in_route = []
        if path_ready == 1:
            if start_visualizing:
                for i in range(1, len(path)):
                    time_step = 0.002
                    current_time = 0

                    prev_state = path[i-1]
                    state = path[i]
                    while current_time < state.dt:
                        current_time += time_step
                        dangle, dx, dy = prev_state.get_steps(current_time, state.psi, state.v, prev_state.angle)
                        x = prev_state.x + dx
                        y = prev_state.y + dy
                        angle = prev_state.angle + dangle
                        points_in_route.append((x,y,angle))

                        pygame.event.pump()
                        render_all(screen, clock, car, obstacles, explored_states, points_in_route)

                path_ready = 0
                stop = 1
        elif not stop:
            render_all(screen, clock, car, obstacles, explored_states, [])

    # Finish execution
    pygame.quit()

if __name__ == "__main__":
    main()
