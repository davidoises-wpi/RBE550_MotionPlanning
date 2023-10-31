import pygame

class Obstacle():
    def __init__(self, position, width, height, color) -> None:

        self.width = width
        self.height = height

        self.x = position[0]
        self.y = position[1]

        self.color = color

        self.surface = pygame.Surface((width, height))
        self.surface.fill(color)
        self.rect = self.surface.get_rect()
        self.rect.center = (self.x, self.y)
        self.mask = pygame.mask.from_surface(self.surface)

    def render(self, parent_surface):
        self.surface.fill(self.color)
        parent_surface.blit(self.surface, self.rect)

    def check_collision(self, obstacles, check_mask):
        collision = False
        for obstacle in obstacles:
            if collision == False:
                if self.rect.colliderect(obstacle.rect):
                    if check_mask:
                        if self.mask.overlap(obstacle.mask, (obstacle.rect[0]-self.rect[0], obstacle.rect[1]-self.rect[1])):
                            collision = True
                    else:
                        collision = True

        return collision
