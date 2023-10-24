import pygame

class SimpleVehicleSprite():
    def __init__(self, img_file, orientation_offset, initial_pos):
        self.orientation_offset = orientation_offset
        self.orientation = 0

        self.x = initial_pos[0]
        self.y = initial_pos[1]

        self.image = pygame.image.load(img_file).convert_alpha()
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
            if self.rect.colliderect(obstacle.rect):
                if self.mask.overlap(obstacle.mask, (obstacle.rect[0]-self.rect[0], obstacle.rect[1]-self.rect[1])):
                    collision = True

        return collision
