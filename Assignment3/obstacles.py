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
        parent_surface.blit(self.surface, self.rect)
