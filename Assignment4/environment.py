from random import random
from obstacles import Obstacle

SCREEN_WIDTH_PIXELS = 750
SCREEN_HEIGHT_PIXELS = SCREEN_WIDTH_PIXELS

SCREEN_WIDTH_METERS = 250

METERS_TO_PIXELS = SCREEN_HEIGHT_PIXELS/SCREEN_WIDTH_METERS

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

world_limit_top = Obstacle((SCREEN_WIDTH_PIXELS/2, -1), SCREEN_WIDTH_PIXELS, 2, GREEN)
world_limit_bot = Obstacle((SCREEN_WIDTH_PIXELS/2, SCREEN_HEIGHT_PIXELS+1), SCREEN_WIDTH_PIXELS, 2, GREEN)
world_limit_left = Obstacle((-1, SCREEN_HEIGHT_PIXELS/2), 2, SCREEN_HEIGHT_PIXELS, GREEN)
world_limit_right = Obstacle((SCREEN_WIDTH_PIXELS+1, SCREEN_HEIGHT_PIXELS/2), 2, SCREEN_HEIGHT_PIXELS, GREEN)

world_limits = []
world_limits.append(world_limit_top)
world_limits.append(world_limit_bot)
world_limits.append(world_limit_left)
world_limits.append(world_limit_right)

class Bush():
    BLOCK_WIDTH = 5*METERS_TO_PIXELS
    BLOCK_HEIGHT = 5*METERS_TO_PIXELS
    BURNING_LIMIT = 100

    def __init__(self, x, y, type) -> None:
        self.blocks = []
        self.burning_level = 0
        if type == 0:
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*2)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
        elif type == 1:
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
        elif type == 2:
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
        else:
            self.blocks.append(Obstacle((x, y-(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x-self.BLOCK_WIDTH, y+(self.BLOCK_HEIGHT*0)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))
            self.blocks.append(Obstacle((x, y+(self.BLOCK_HEIGHT*1)), self.BLOCK_WIDTH, self.BLOCK_HEIGHT, GREEN))

    def render(self, parent_surface):
        for block in self.blocks:
            block.render(parent_surface)

    def check_collisions(self, obstacles):
        for block in self.blocks:
            if block.check_collision(obstacles, True):
                return True

        return False


def  populate_map(density):

    obstacle_list = []
    bush_list = []

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
        bush_list.append(Bush(obstacle.x, obstacle.y, bush_type))

    return bush_list
