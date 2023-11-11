import environment
from states import DotDriveState
import time
from math import radians, sqrt
from random import random

WUMPUS_STEP_PER_CYCLE = 5*environment.METERS_TO_PIXELS

wumpus_ready_for_planning = False
wumpus_initial_state = None
wumpus_goal_state = None

wumpus_path = []
wumpus_path_ready = False

def initialize_wumpus_initial_state(wumpus):
    global wumpus_initial_state

    posx = round(random()*environment.SCREEN_WIDTH_PIXELS)
    posy = round(random()*environment.SCREEN_HEIGHT_PIXELS)

    wumpus_initial_state = DotDriveState(posx, posy, WUMPUS_STEP_PER_CYCLE)
    wumpus.set_position(posx, posy)

def initialize_wumpus_goal_state(wumpus):
    # min_distance = 1000
    min_distance = 0
    goal_x = 0
    goal_y = 0
    for bush in environment.bushes:
        if bush.state == environment.Bush.EXTINGUISHED_STATE or bush.state == environment.Bush.NORMAL_STATE:
            x_diff = bush.x - wumpus.x
            y_diff = bush.y - wumpus.y
            dist = sqrt(x_diff**2 + y_diff**2)
            # if dist < min_distance:
            if dist > min_distance:
                min_distance = dist
                goal_x = bush.x
                goal_y = bush.y

    global wumpus_goal_state
    wumpus_goal_state = DotDriveState(goal_x, goal_y, WUMPUS_STEP_PER_CYCLE)


def is_wumpus_path_ready():
    return wumpus_path_ready

def get_wumpus_path():
    return wumpus_path

def search_path(vehicle, obstacles, end_state):
    max_iterations = 10000

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

            # path = reconstruct_path(search_path.closed_states)
            path = [1, 2, 4]

            print("success")
            print("Finished reconstructing path")
            print("Number of iterations: ", search_path.iterations)
            print("Number of waypoints: ", len(path))
            print("Press any key to visualize the path")

            exit_code = 1
            return exit_code, path, search_path.closed_states
        else:
            search_path.iterations += 1
            neighbors = current_state.get_neighbors(vehicle, obstacles, search_path.open_states, search_path.closed_states, end_state, (0, environment.SCREEN_WIDTH_PIXELS), (0, environment.SCREEN_HEIGHT_PIXELS))
            search_path.open_states += neighbors
            exit_code = 0
            return exit_code, [], search_path.closed_states
    else:
        print("Solution not found")
        exit_code = -1
        return exit_code, [], search_path.closed_states

def wumpus_main(wumpus):
    global wumpus_path, wumpus_path_ready, wumpus_initial_state, wumpus_ready_for_planning
    global open_states, closed_states, iterations

    while True:
        if not wumpus_ready_for_planning:
            wumpus_ready_for_planning = True
            wumpus_initial_state = DotDriveState(wumpus.x, wumpus.y, WUMPUS_STEP_PER_CYCLE)
            initialize_wumpus_goal_state(wumpus)

            search_path.open_states = [wumpus_initial_state]
            search_path.closed_states = []
            search_path.iterations = 0

            print("Initial state x: ", wumpus_initial_state.x, " y: ", wumpus_initial_state.y)
            print("Goal state x: ", wumpus_goal_state.x, " y: ", wumpus_goal_state.y)
        elif not wumpus_path_ready:
            wumpus_path_ready, wumpus_path, explored_states = search_path(wumpus, environment.bushes, wumpus_goal_state)
        else:
            print("Path found")
            # wumpus.set_position(i, 300)
            time.sleep(1)
            #TODO logic to execute the steps in the found path
