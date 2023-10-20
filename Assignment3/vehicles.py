import pygame

class SimpleVehicleSprite():
    def __init__(self, img_file, orientation_offset, initial_pos):
        self.orientation_offset = orientation_offset
        self.orientation = 0

        self.x = initial_pos[0]
        self.y = initial_pos[0]

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

    def render(self, parent_surface):
        self.rect.center = (self.x, self.y)
        self.surface = pygame.transform.rotate(self.image, self.orientation + self.orientation_offset)
        self.rect = self.surface.get_rect()
        self.rect.center = (self.x, self.y)
        self.mask = pygame.mask.from_surface(self.surface)

        parent_surface.blit(self.surface, self.rect)
