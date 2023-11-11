import environment
from states import DotDriveState
import time
from math import radians
from random import random

wumpus_path = []
wumpus_path_ready = False

wumpus_ready_for_planning = False
wumpus_initial_state = None
wumpus_goal_state = None

def initialize_wumpus_initial_state(wumpus):
    global wumpus_initial_state

    posx = round(random()*environment.SCREEN_WIDTH_PIXELS)
    posy = round(random()*environment.SCREEN_HEIGHT_PIXELS)

    wumpus_initial_state = DotDriveState(posx, posy, 5*environment.METERS_TO_PIXELS)
    wumpus.set_position(posx, posy)

def initialize_wumpus_goal_state():
    pass
# TODO logic here to find the next bush to be the goal


def is_wumpus_path_ready():
    return wumpus_path_ready

def get_wumpus_path():
    return wumpus_path

def wumpus_main(wumpus):
    global wumpus_path, wumpus_path_ready

    while True:
        if not wumpus_ready_for_planning:
            initialize_wumpus_goal_state()
            # TODO logic here to upadte initial state and goal state
        else:
            wumpus_path = [1,2,3]
            wumpus_path_ready = False
            for i in range(600):
                time.sleep(0.01)
                wumpus.set_position(i, 300)
            wumpus_path_ready = True
            time.sleep(1)
