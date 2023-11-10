import time

wumpus_path = []
wumpus_path_ready = False

def is_wumpus_path_ready():
    return wumpus_path_ready

def get_wumpus_path():
    return wumpus_path

def wumpus_main(wumpus):
    while True:
        global wumpus_path, wumpus_path_ready
        wumpus_path = [1,2,3]
        global wumpus_path_ready
        wumpus_path_ready = False
        for i in range(600):
            time.sleep(0.01)
            wumpus.set_position(i, 300)
        wumpus_path_ready = True
