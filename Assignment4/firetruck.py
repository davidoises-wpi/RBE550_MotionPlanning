import environment
from states import AckermannDriveState
import time
from math import radians, sqrt, degrees
from random import random
import copy

FIRETRUCK_STEP_PER_CYCLE = 5*environment.METERS_TO_PIXELS

stop_playing = False
firetruck_ready_for_planning = False
firetruck_initial_state = None
firetruck_goal_state = None
goal_bush = None

firetruck_explored_states = []
firetruck_path = []
firetruck_route = []
firetruck_path_ready = False

firetruck_path_state_index = 0

planning_start_time = 0
total_planning_time = 0

dt = 0.5
visualization_time_step = 0.02

prm_nodes = []
prm_connections = []
prm_n = 500
prm_k = 7
prm_state = 0
prm_index = 0
neighbor_index = 0
neighbors = []

def initialize_firetruck_initial_state(firetruck, wumpus):
    global firetruck_initial_state

    posx = round(environment.SCREEN_WIDTH_PIXELS - wumpus.x)
    posy = round(environment.SCREEN_HEIGHT_PIXELS - wumpus.y)

    firetruck_initial_state = AckermannDriveState(posx, posy, 0, 0, 0, 0)
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

            # metric = bush.burning_level + 1.0/dist
            metric = dist
            if metric > best_match:
                best_match = metric
                goal_x = bush.x
                goal_y = bush.y
                goal_bush = bush

    global firetruck_goal_state
    firetruck_goal_state = AckermannDriveState(goal_x, goal_y, 0, 0, 0, 0)

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

def build_map(vehicle, obstacles):
    global prm_nodes,prm_index, prm_state, neighbor_index, neighbors, prm_n
    if prm_index < prm_n and prm_state == 0:
        x = random()*environment.SCREEN_WIDTH_PIXELS
        y = random()*environment.SCREEN_HEIGHT_PIXELS
        angle = radians(random()*360.0)

        new_state = AckermannDriveState(x, y, angle, 0, 0, 0)

        if new_state not in prm_nodes:

            temp_vehicle = copy.deepcopy(vehicle)
            temp_vehicle.set_position(x, y)
            temp_vehicle.set_orientation(degrees(angle))
            temp_vehicle.update()

            colided = False
            for obstacle in obstacles:
                colided = obstacle.check_collisions([temp_vehicle])
                if colided:
                    break

            if not colided:
                prm_nodes.append(new_state)
                prm_index += 1

    if prm_index == prm_n and prm_state == 0:
        print("Connecting sample nodes")
        prm_state = 1
        prm_index = 0

    if prm_index == prm_n and prm_state == 1:
        print("PRM ready")
        return 1

    if prm_index < prm_n and prm_state == 1:

        ret = connect_node(vehicle, obstacles, prm_nodes[prm_index], prm_k, False)
        if ret == -1:
            print("issue")
            del prm_nodes[prm_index]
            prm_n -= 1

        # if neighbor_index == prm_k:
        if ret == 1:
            print(prm_index)
            prm_index += 1

def connect_node(vehicle, obstacles, new_node, k_neighbors, stop_if_exists):
    neighbor_index = 0
    neighbor_node = None
    neighbors = []

    if new_node in prm_nodes and stop_if_exists:
        # Already connected
        return 1

    while neighbor_index < k_neighbors:
        # Find k nearest neighbors
        lowest_distance = 10000
        for node in prm_nodes:
            if not node == new_node:
                if node not in neighbors:
                    distance = node.calculate_euclidean_distance(new_node)
                    if distance < lowest_distance:
                        lowest_distance = distance
                        neighbor_node = node
        neighbors.append(neighbor_node)

        local_search.open_states = [new_node]
        local_search.closed_states = []
        local_search.iterations = 0

        path = []
        explored = []

        path_ready = 0
        while path_ready == 0:
            # Continue searching
            path_ready, path, explored = local_search(vehicle, obstacles, neighbor_node)

        if path_ready == 1:
            prm_connections.append((new_node, neighbor_node, path))
            neighbor_index += 1

        if path_ready == -1:
            print(distance, " ", len(neighbors), " ", neighbor_index)

        if len(neighbors) > 10 and neighbor_index == 0:
            # Couldnt connect it
            return -1

        if len(neighbors) > 20 and neighbor_index > 0:
            # At least we made some connections
            return 1
    # print("loop finished")
    return 1

def prm_search(vehicle, obstacles, start_state, end_state):
    ret = connect_node(vehicle, obstacles, start_state, 1, True)
    if ret == -1:
        # Could not connect the initial state
        return -1

    ret = connect_node(vehicle, obstacles, end_state, 1, True)
    if ret == -1:
        # Could not connect the final state
        return -1


def local_search(vehicle, obstacles, end_state):
    max_iterations = 10000

    # Actual planning
    if local_search.open_states and local_search.iterations < max_iterations:

        # Search and remove the state with the lowest cost
        current_state = local_search.open_states[0]

        state_index = 0
        i = 0
        for state in local_search.open_states:
            if state.total_cost < current_state.total_cost:
                current_state = state
                state_index = i
            i += 1
        del local_search.open_states[state_index]

        # Add the current state to the closed list
        local_search.closed_states.append(current_state)

        if current_state.goal_check(end_state):

            path = reconstruct_path(local_search.closed_states)

            # print("success")
            # print("Finished reconstructing path")
            # print("Number of iterations: ", local_search.iterations)
            # print("Number of waypoints: ", len(path))

            exit_code = 1
            return exit_code, path, local_search.closed_states
        else:
            local_search.iterations += 1
            neighbors = current_state.get_neighbors(vehicle, obstacles, dt, local_search.open_states, local_search.closed_states, end_state, (0, environment.SCREEN_WIDTH_PIXELS), (0, environment.SCREEN_HEIGHT_PIXELS))
            local_search.open_states += neighbors
            exit_code = 0
            return exit_code, [], local_search.closed_states
    else:
        print("Solution not found")
        exit_code = -1
        return exit_code, [], local_search.closed_states

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
                firetruck_initial_state = AckermannDriveState(firetruck.x, firetruck.y, radians(firetruck.orientation), 0, 0, 0)
                initialize_firetruck_goal_state(firetruck)

                local_search.open_states = [firetruck_initial_state]
                local_search.closed_states = []
                local_search.iterations = 0

                firetruck_route = []
                firetruck_path = []

                # print("Initial state x: ", firetruck_initial_state.x, " y: ", firetruck_initial_state.y)
                # print("Goal state x: ", firetruck_goal_state.x, " y: ", firetruck_goal_state.y)

                planning_start_time = time.time()
            elif firetruck_path_ready == 0:
                # Continue searching
                firetruck_path_ready, firetruck_path, firetruck_explored_states = local_search(firetruck, environment.bushes, firetruck_goal_state)
            elif firetruck_path_ready == -1:
                # Solution not found
                firetruck_path_state_index = 0
                firetruck_path_ready = False
                firetruck_ready_for_planning = False
            else:
                if firetruck_path_ready == 1:
                    planning_time = time.time() - planning_start_time
                    total_planning_time += planning_time

                    # create a smooth line between nodes base on steering angle and speed
                    firetruck_route = []
                    for i in range(1, len(firetruck_path)):
                        visualization_time_step = 0.02
                        current_time = 0

                        prev_state = firetruck_path[i-1]
                        state = firetruck_path[i]
                        while current_time < state.dt:
                            current_time += visualization_time_step
                            dangle, dx, dy = prev_state.get_steps(current_time, state.psi, state.v, prev_state.angle)
                            x = prev_state.x + dx
                            y = prev_state.y + dy
                            angle = prev_state.angle + dangle
                            firetruck_route.append((x,y,angle))

                    firetruck_path_ready = 2
                # solution found
                if firetruck_path_state_index < len(firetruck_route):
                    state = firetruck_route[firetruck_path_state_index]
                    firetruck.set_position(state[0], state[1])
                    firetruck.set_orientation(degrees(state[2]))

                    real_time_millis = visualization_time_step/(environment.REAL_TO_SIMULATION_SECS_PER_MILLIS*1000)
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
