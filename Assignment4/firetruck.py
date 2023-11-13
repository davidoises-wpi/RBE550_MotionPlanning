import environment
from states import DotDriveState
import time
from math import radians, sqrt
from random import random

FIRETRUCK_STEP_PER_CYCLE = 5*environment.METERS_TO_PIXELS

stop_playing = False
firetruck_ready_for_planning = False
firetruck_initial_state = None
firetruck_goal_state = None
goal_bush = None

firetruck_explored_states = []
firetruck_path = []
firetruck_path_ready = False

firetruck_path_state_index = 0

planning_start_time = 0
total_planning_time = 0

def initialize_firetruck_initial_state(firetruck, wumpus):
    global firetruck_initial_state

    posx = round(environment.SCREEN_WIDTH_PIXELS - wumpus.x)
    posy = round(environment.SCREEN_HEIGHT_PIXELS - wumpus.y)

    firetruck_initial_state = DotDriveState(posx, posy, FIRETRUCK_STEP_PER_CYCLE)
    firetruck.set_position(posx, posy)

def initialize_firetruck_goal_state(firetruck):
    global goal_bush, stop_playing

    best_match = 0
    goal_x = 0
    goal_y = 0
    for bush in environment.bushes:
        if bush.state == environment.Bush.BURNING_STATE or bush.state == environment.Bush.EXPANDING_STATE:
            x_diff = bush.x - firetruck.x
            y_diff = bush.y - firetruck.y
            dist = sqrt(x_diff**2 + y_diff**2)

            metric = bush.burning_level + 1.0/dist
            if metric > best_match:
                best_match = metric
                goal_x = bush.x
                goal_y = bush.y
                goal_bush = bush

    global firetruck_goal_state
    firetruck_goal_state = DotDriveState(goal_x, goal_y, FIRETRUCK_STEP_PER_CYCLE)
    # print(goal_bush.state)


def is_firetruck_path_ready():
    return (firetruck_path_ready == 1)

def get_firetruck_path():
    return firetruck_path

def get_firetruck_explored_states():
    return firetruck_explored_states

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

def firetruck_main(firetruck):
    global firetruck_path, firetruck_path_ready, firetruck_initial_state, firetruck_ready_for_planning
    global firetruck_explored_states
    global firetruck_path_state_index
    global goal_bush
    global planning_start_time, total_planning_time

    while True:
        all_bushes_burned = True
        for bush in environment.bushes:
            if bush.state == environment.Bush.BURNING_STATE or bush.state == environment.Bush.EXPANDING_STATE:
                all_bushes_burned = False
        if all_bushes_burned:
            firetruck_path_state_index = 0
            firetruck_path_ready = False
            firetruck_ready_for_planning = False

        if stop_playing:
            print("PRM Planning time: ", total_planning_time)
            break

        if not all_bushes_burned and not stop_playing:
            if not firetruck_ready_for_planning:
                firetruck_ready_for_planning = True
                firetruck_initial_state = DotDriveState(firetruck.x, firetruck.y, FIRETRUCK_STEP_PER_CYCLE)
                initialize_firetruck_goal_state(firetruck)

                search_path.open_states = [firetruck_initial_state]
                search_path.closed_states = []
                search_path.iterations = 0

                # print("Initial state x: ", firetruck_initial_state.x, " y: ", firetruck_initial_state.y)
                # print("Goal state x: ", firetruck_goal_state.x, " y: ", firetruck_goal_state.y)

                planning_start_time = time.time()
            elif firetruck_path_ready == 0:
                # Continue searching
                firetruck_path_ready, firetruck_path, firetruck_explored_states = search_path(firetruck, environment.bushes, firetruck_goal_state)
            elif firetruck_path_ready == -1:
                # Solution not found
                firetruck_path_state_index = 0
                firetruck_path_ready = False
                firetruck_ready_for_planning = False
            else:
                if firetruck_path_ready == 1:
                    planning_time = time.time() - planning_start_time
                    total_planning_time += planning_time
                    firetruck_path_ready = 2
                # solution found
                if firetruck_path_state_index < len(firetruck_path):
                    state = firetruck_path[firetruck_path_state_index]
                    firetruck.set_position(state.x, state.y)

                    # This defines the speed at which wumpus moves
                    # the grid for wumpus is 5 meters, so it is moving 5 meters ever 1.5 secs in simulation time
                    # speed = 5/1.5 = 3.3m/s
                    simulation_time_between_steps = 0.5
                    # simulation_time_between_steps = 0.1
                    real_time_millis = simulation_time_between_steps/(environment.REAL_TO_SIMULATION_SECS_PER_MILLIS*1000)
                    time.sleep(real_time_millis)

                    firetruck_path_state_index += 1
                else:
                    # wait 5 seconds in simulation time
                    real_time_millis = 5/(environment.REAL_TO_SIMULATION_SECS_PER_MILLIS*1000)
                    time.sleep(real_time_millis)

                    goal_bush.extinguished = True
                    goal_bush.touched = False

                    firetruck_path_state_index = 0
                    firetruck_path_ready = False
                    firetruck_ready_for_planning = False
