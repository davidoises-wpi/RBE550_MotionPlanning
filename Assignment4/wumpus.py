import environment
from states import DotDriveState
import time
from math import radians, sqrt
from random import random

WUMPUS_STEP_PER_CYCLE = 5*environment.METERS_TO_PIXELS

stop_playing = False
wumpus_ready_for_planning = False
wumpus_initial_state = None
wumpus_goal_state = None
goal_bush = None

wumpus_explored_states = []
wumpus_path = []
wumpus_path_ready = False

wumpus_path_state_index = 0

planning_start_time = 0

def initialize_wumpus_initial_state(wumpus):
    global wumpus_initial_state

    posx = round(random()*environment.SCREEN_WIDTH_PIXELS)
    posy = round(random()*environment.SCREEN_HEIGHT_PIXELS)

    wumpus_initial_state = DotDriveState(posx, posy, WUMPUS_STEP_PER_CYCLE)
    wumpus.set_position(posx, posy)

def initialize_wumpus_goal_state(wumpus):
    global goal_bush, stop_playing

    min_distance = 1000
    # min_distance = 0
    goal_x = 0
    goal_y = 0
    for bush in environment.bushes:
        if bush.state == environment.Bush.EXTINGUISHED_STATE or bush.state == environment.Bush.NORMAL_STATE:
            x_diff = bush.x - wumpus.x
            y_diff = bush.y - wumpus.y
            dist = sqrt(x_diff**2 + y_diff**2)
            if dist < min_distance:
            # if dist > min_distance:
                min_distance = dist
                goal_x = bush.x
                goal_y = bush.y
                goal_bush = bush

    global wumpus_goal_state
    wumpus_goal_state = DotDriveState(goal_x, goal_y, WUMPUS_STEP_PER_CYCLE)


def is_wumpus_path_ready():
    return (wumpus_path_ready == 1)

def get_wumpus_path():
    return wumpus_path

def get_wumpus_explored_states():
    return wumpus_explored_states

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

            # print("success")
            # print("Finished reconstructing path")
            # print("Number of iterations: ", search_path.iterations)
            # print("Number of waypoints: ", len(path))

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
    global wumpus_explored_states
    global wumpus_path_state_index
    global goal_bush
    global planning_start_time

    while True:
        all_bushes_burned = True
        for bush in environment.bushes:
            if bush.state == environment.Bush.NORMAL_STATE or bush.state == environment.Bush.EXTINGUISHED_STATE:
                all_bushes_burned = False
        if all_bushes_burned:
            wumpus_path_state_index = 0
            wumpus_path_ready = False
            wumpus_ready_for_planning = False

        if not all_bushes_burned and not stop_playing:
            if not wumpus_ready_for_planning:
                wumpus_ready_for_planning = True
                wumpus_initial_state = DotDriveState(wumpus.x, wumpus.y, WUMPUS_STEP_PER_CYCLE)
                initialize_wumpus_goal_state(wumpus)

                search_path.open_states = [wumpus_initial_state]
                search_path.closed_states = []
                search_path.iterations = 0

                # print("Initial state x: ", wumpus_initial_state.x, " y: ", wumpus_initial_state.y)
                # print("Goal state x: ", wumpus_goal_state.x, " y: ", wumpus_goal_state.y)

                planning_start_time = time.time()
            elif wumpus_path_ready == 0:
                # Continue searching
                wumpus_path_ready, wumpus_path, wumpus_explored_states = search_path(wumpus, environment.bushes, wumpus_goal_state)
            elif wumpus_path_ready == -1:
                # Solution not found
                wumpus_path_state_index = 0
                wumpus_path_ready = False
                wumpus_ready_for_planning = False
            else:
                if wumpus_path_ready == 1:
                    planning_time = time.time() - planning_start_time
                    print("A* Planning time: ", planning_time)
                    wumpus_path_ready = 2
                # solution found
                if wumpus_path_state_index < len(wumpus_path):
                    state = wumpus_path[wumpus_path_state_index]
                    wumpus.set_position(state.x, state.y)

                    # This defines the speed at which wumpus moves
                    # the grid for wumpus is 5 meters, so it is moving 5 meters ever 1.5 secs in simulation time
                    # speed = 5/1.5 = 3.3m/s
                    simulation_time_between_steps = 2.5
                    # simulation_time_between_steps = 0.1
                    real_time_millis = simulation_time_between_steps/(environment.REAL_TO_SIMULATION_SECS_PER_MILLIS*1000)
                    time.sleep(real_time_millis)

                    wumpus_path_state_index += 1
                else:
                    goal_bush.extinguished = False
                    goal_bush.touched = True

                    wumpus_path_state_index = 0
                    wumpus_path_ready = False
                    wumpus_ready_for_planning = False
