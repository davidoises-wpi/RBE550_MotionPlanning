import environment
from states import DotDriveState
import time
from math import radians, sqrt
from random import random

WUMPUS_STEP_PER_CYCLE = 5*environment.METERS_TO_PIXELS

wumpus_path = []
wumpus_path_ready = False

wumpus_ready_for_planning = False
wumpus_initial_state = None
wumpus_goal_state = None

def initialize_wumpus_initial_state(wumpus):
    global wumpus_initial_state

    posx = round(random()*environment.SCREEN_WIDTH_PIXELS)
    posy = round(random()*environment.SCREEN_HEIGHT_PIXELS)

    wumpus_initial_state = DotDriveState(posx, posy, WUMPUS_STEP_PER_CYCLE)
    wumpus.set_position(posx, posy)

def initialize_wumpus_goal_state(wumpus):
    min_distance = 1000
    goal_x = 0
    goal_y = 0
    for bush in environment.bushes:
        if bush.state == environment.Bush.EXTINGUISHED_STATE or bush.state == environment.Bush.NORMAL_STATE:
            x_diff = bush.x - wumpus.x
            y_diff = bush.y - wumpus.y
            dist = sqrt(x_diff**2 + y_diff**2)
            if dist < min_distance:
                min_distance = dist
                goal_x = bush.x
                goal_y = bush.y

    global wumpus_goal_state
    wumpus_goal_state = DotDriveState(goal_x, goal_y, WUMPUS_STEP_PER_CYCLE)


def is_wumpus_path_ready():
    return wumpus_path_ready

def get_wumpus_path():
    return wumpus_path

def wumpus_main(wumpus):
    global wumpus_path, wumpus_path_ready, wumpus_initial_state, wumpus_ready_for_planning

    while True:
        if not wumpus_ready_for_planning:
            wumpus_ready_for_planning = True
            wumpus_initial_state = DotDriveState(wumpus.x, wumpus.y, WUMPUS_STEP_PER_CYCLE)
            initialize_wumpus_goal_state(wumpus)
            print("Initial state x: ", wumpus_initial_state.x, " y: ", wumpus_initial_state.y)
            print("Goal state x: ", wumpus_goal_state.x, " y: ", wumpus_goal_state.y)
        elif not wumpus_path_ready:
            wumpus_path = [1,2,3]
            wumpus_path_ready = False
            #TODO plan
            for i in range(600):
                time.sleep(0.01)
                # wumpus.set_position(i, 300)
            wumpus_path_ready = True
            time.sleep(1)
        else:
            pass
            #TODO logic to execute the steps in the found path
