from random import random
from obstacles import Obstacle
from math import sqrt

SCREEN_WIDTH_PIXELS = 750
SCREEN_HEIGHT_PIXELS = SCREEN_WIDTH_PIXELS

SCREEN_WIDTH_METERS = 250

METERS_TO_PIXELS = SCREEN_HEIGHT_PIXELS/SCREEN_WIDTH_METERS

# TOTAL_REAL_TIME_SEC = 1800
TOTAL_REAL_TIME_SEC = 450
# TOTAL_REAL_TIME_SEC = 10

# TOTAL_REAL_TIME_SEC = 120
TOTAL_SIMULATION_TIME_SEC = 3600
REAL_TO_SIMULATION_SECS_PER_MILLIS = TOTAL_SIMULATION_TIME_SEC/(TOTAL_REAL_TIME_SEC*1000.0)

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GREY = (10, 10, 10)
PURPLE = (128, 0, 128)

world_limit_top = Obstacle((SCREEN_WIDTH_PIXELS/2, -1), SCREEN_WIDTH_PIXELS, 2, GREEN)
world_limit_bot = Obstacle((SCREEN_WIDTH_PIXELS/2, SCREEN_HEIGHT_PIXELS+1), SCREEN_WIDTH_PIXELS, 2, GREEN)
world_limit_left = Obstacle((-1, SCREEN_HEIGHT_PIXELS/2), 2, SCREEN_HEIGHT_PIXELS, GREEN)
world_limit_right = Obstacle((SCREEN_WIDTH_PIXELS+1, SCREEN_HEIGHT_PIXELS/2), 2, SCREEN_HEIGHT_PIXELS, GREEN)

world_limits = []
world_limits.append(world_limit_top)
world_limits.append(world_limit_bot)
world_limits.append(world_limit_left)
world_limits.append(world_limit_right)

bushes = []

class Bush():
    BLOCK_WIDTH = 5*METERS_TO_PIXELS
    BLOCK_HEIGHT = 5*METERS_TO_PIXELS
    BURNING_LIMIT = 100
    BURNING_STEP = TOTAL_REAL_TIME_SEC/TOTAL_SIMULATION_TIME_SEC

    BURNING_STATE = 0
    BURNED_STATE = 1
    EXTINGUISHED_STATE = 2
    NORMAL_STATE = 3
    EXPANDING_STATE = 4

    def __init__(self, x, y, type) -> None:
        self.blocks = []
        self.burning_level = 0
        self.burning_time = 0
        self.burning_start_time = 0
        self.state = self.NORMAL_STATE
        self.touched = False
        self.extinguished = False
        self.expanded = False
        if type == 0:
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*2)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))

            # Approximate centroid based on the shape of the figure
            self.x = x
            self.y = y-self.BLOCK_HEIGHT/2.0
        elif type == 1:
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))

            # Approximate centroid based on the shape of the figure
            self.x = x-self.BLOCK_WIDTH/2.0
            self.y = y
        elif type == 2:
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            # Approximate centroid based on the shape of the figure
            self.x = x-self.BLOCK_WIDTH/2.0
            self.y = y
        else:
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            # Approximate centroid based on the shape of the figure
            self.x = x-self.BLOCK_WIDTH/2.0
            self.y = y

    def render(self, parent_surface):
        for block in self.blocks:
            if self.state == self.BURNED_STATE:
                block.color = GREY
            elif self.state == self.BURNING_STATE:
                block.color = RED
            elif self.state == self.EXPANDING_STATE:
                block.color = PURPLE
            else:
                block.color = GREEN
            block.render(parent_surface)

    def check_collisions(self, obstacles):
        for block in self.blocks:
            if block.check_collision(obstacles, True):
                return True

        return False


def  populate_map(density):

    obstacle_list = []

    number_of_obstacles = density*SCREEN_WIDTH_PIXELS*SCREEN_HEIGHT_PIXELS/(Bush.BLOCK_WIDTH*Bush.BLOCK_HEIGHT*4)
    number_of_obstacles = round(number_of_obstacles)

    for i in range(number_of_obstacles):
        success = False
        retry_count = 0
        while not success and retry_count < 20:
            retry_count += 1
            posx = round(random()*SCREEN_WIDTH_PIXELS)
            posy = round(random()*SCREEN_HEIGHT_PIXELS)

            new_obstacle = Obstacle((posx, posy), 4*Bush.BLOCK_WIDTH, 6*Bush.BLOCK_HEIGHT, RED)
            if not new_obstacle.check_collision(obstacle_list, False) and not new_obstacle.check_collision(world_limits, False):
                success = True
                obstacle_list.append(new_obstacle)

    for obstacle in obstacle_list:
        bush_type = round(random()*3.5)
        bushes.append(Bush(obstacle.x, obstacle.y, bush_type))

def update_environment(firetruck, wumpus, time):
    for bush in bushes:
        touched = bush.check_collisions([wumpus])

        # Bush is touched if another bush had already set its flag or if wumpus was too close
        bush.touched = bush.touched or touched

        # Pending to implement logic based on firetruck
        bush.extinguished = False

        # Bushes state machine transitions
        if bush.state == Bush.NORMAL_STATE:
            if bush.touched:
                bush.state = Bush.BURNING_STATE
        elif bush.state == Bush.EXTINGUISHED_STATE:
            if bush.touched:
                bush.state = Bush.BURNING_STATE
        elif bush.state == Bush.BURNING_STATE:
            if bush.burning_time >= 10.0 and bush.expanded == False:
                bush.state = Bush.EXPANDING_STATE
            elif bush.burning_level >= Bush.BURNING_LIMIT:
                bush.state = Bush.BURNED_STATE
            elif bush.extinguished:
                bush.state = Bush.EXTINGUISHED_STATE
        elif bush.state == Bush.EXPANDING_STATE:
            if bush.burning_time >= 13.0:
                bush.state = Bush.BURNING_STATE
        else: #bush.state == Bush.BURNED_STATE:
            bush.state = Bush.BURNED_STATE

        # State actions
        if bush.state == Bush.NORMAL_STATE:
            bush.burning_start_time = time
            bush.burning_level = 0
            bush.touched = False
            bush.expanded = False
        elif bush.state == Bush.EXTINGUISHED_STATE:
            bush.burning_start_time = time
            bush.touched = False
            bush.expanded = False
        elif bush.state == Bush.BURNING_STATE:
            bush.burning_time = time - bush.burning_start_time
            bush.burning_level += bush.BURNING_STEP
        elif bush.state == Bush.EXPANDING_STATE:
            bush.burning_time = time - bush.burning_start_time
            bush.burning_level += bush.BURNING_STEP
            # Start expanding fire here
            for neighbor_bush in bushes:
                if neighbor_bush.state == Bush.NORMAL_STATE or neighbor_bush.state == Bush.EXTINGUISHED_STATE:
                    x_diff = neighbor_bush.x - bush.x
                    y_diff = neighbor_bush.y - bush.y

                    dist = sqrt(x_diff**2 + y_diff**2)
                    dist = dist / METERS_TO_PIXELS

                    if dist <= 30.0:
                        neighbor_bush.touched = True

            bush.expanded = True
        else: #bush.state == Bush.BURNED_STATE:
            pass
