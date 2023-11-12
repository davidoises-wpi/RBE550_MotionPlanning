import pygame
from math import cos, sin, radians

class SimpleVehicleSprite():
    def __init__(self, img_file, orientation_offset, initial_pos, scaling=None):
        self.orientation_offset = orientation_offset
        self.orientation = 0

        self.x = initial_pos[0]
        self.y = initial_pos[1]

        self.img_file = img_file
        self.scaling = scaling

        self.image = pygame.image.load(img_file).convert_alpha()
        if scaling:
            self.image = pygame.transform.scale(self.image, scaling)
        self.surface = pygame.transform.rotate(self.image, self.orientation_offset)
        self.rect = self.surface.get_rect()
        self.mask = pygame.mask.from_surface(self.surface)

        self.rect.center = (self.x, self.y)

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def set_orientation(self, angle):
        self.orientation = angle


    def update(self):
        self.rect.center = (self.x, self.y)
        self.surface = pygame.transform.rotate(self.image, self.orientation + self.orientation_offset)
        self.rect = self.surface.get_rect()
        self.rect.center = (self.x, self.y)
        self.mask = pygame.mask.from_surface(self.surface)

    def render(self, parent_surface):
        self.update()
        parent_surface.blit(self.surface, self.rect)

    def check_collision(self, obstacles):
        collision = False
        for obstacle in obstacles:
            if collision == False:
                if self.rect.colliderect(obstacle.rect):
                    if self.mask.overlap(obstacle.mask, (obstacle.rect[0]-self.rect[0], obstacle.rect[1]-self.rect[1])):
                        collision = True

        return collision

    def __deepcopy__(self, soemthing_else):
        return SimpleVehicleSprite(self.img_file, self.orientation_offset, (self.x, self.y), self.scaling)


class TrailerVehicleSprite():
    DISTANCE_AXLES_M = 5

    WHEELBASE_M = 3

    WHEELBASE_PIXELS = 86


    def __init__(self, img_file, trailer_img_file, orientation_offset, trailer_orientation_offset, initial_pos):
        self.orientation_offset = orientation_offset
        self.orientation = 0

        self.trailer_orientation_offset = trailer_orientation_offset
        self.trailer_orientation = 0

        self.x = initial_pos[0]
        self.y = initial_pos[1]

        x_offset = self.DISTANCE_AXLES_M*cos(self.trailer_orientation)*self.WHEELBASE_PIXELS/self.WHEELBASE_M
        y_offset = self.DISTANCE_AXLES_M*sin(self.trailer_orientation)*self.WHEELBASE_PIXELS/self.WHEELBASE_M
        self.trailer_x = self.x - x_offset
        self.trailer_y = self.y - y_offset

        self.image = pygame.image.load(img_file).convert_alpha()
        self.trailer_image = pygame.image.load(trailer_img_file).convert_alpha()

        self.surface = pygame.transform.rotate(self.image, self.orientation_offset)
        self.rect = self.surface.get_rect()
        self.mask = pygame.mask.from_surface(self.surface)
        self.rect.center = (self.x, self.y)

        self.trailer_surface = pygame.transform.rotate(self.trailer_image, self.trailer_orientation_offset)
        self.trailer_rect = self.trailer_surface.get_rect()
        self.trailer_mask = pygame.mask.from_surface(self.trailer_surface)
        self.trailer_rect.center = (self.trailer_x, self.trailer_y)

    def set_position(self, x, y):
        self.x = x
        self.y = y

        x_offset = self.DISTANCE_AXLES_M*cos(radians(self.trailer_orientation))*self.WHEELBASE_PIXELS/self.WHEELBASE_M
        y_offset = self.DISTANCE_AXLES_M*sin(radians(self.trailer_orientation))*self.WHEELBASE_PIXELS/self.WHEELBASE_M
        self.trailer_x = self.x - x_offset
        self.trailer_y = self.y + y_offset

    def set_orientation(self, angle, trailer_angle):
        self.orientation = angle
        self.trailer_orientation = trailer_angle

    def update(self):
        self.rect.center = (self.x, self.y)
        self.trailer_rect.center = (self.trailer_x, self.trailer_y)

        self.surface = pygame.transform.rotate(self.image, self.orientation + self.orientation_offset)
        self.rect = self.surface.get_rect()
        self.rect.center = (self.x, self.y)
        self.mask = pygame.mask.from_surface(self.surface)

        self.trailer_surface = pygame.transform.rotate(self.trailer_image, self.trailer_orientation + self.trailer_orientation_offset)
        self.trailer_rect = self.trailer_surface.get_rect()
        self.trailer_rect.center = (self.trailer_x, self.trailer_y)
        self.trailer_mask = pygame.mask.from_surface(self.trailer_surface)

    def render(self, parent_surface):
        self.update()
        parent_surface.blit(self.surface, self.rect)
        parent_surface.blit(self.trailer_surface, self.trailer_rect)

    def check_collision(self, obstacles):
        collision = False

        for obstacle in obstacles:
            if collision == False:
                if self.rect.colliderect(obstacle.rect):
                    if self.mask.overlap(obstacle.mask, (obstacle.rect[0]-self.rect[0], obstacle.rect[1]-self.rect[1])):
                        collision = True

                if self.trailer_rect.colliderect(obstacle.rect):
                    if self.trailer_mask.overlap(obstacle.mask, (obstacle.rect[0]-self.trailer_rect[0], obstacle.rect[1]-self.trailer_rect[1])):
                        collision = True

        return collision
