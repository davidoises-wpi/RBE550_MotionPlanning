import pygame

# The window dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 800

# Some basic definitions of colors
RED = (255,0,0)
GREEN = (0,255,0)
YELLOW = (255,255,0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Cell:
    # Cell type constant strings
    EMPTY = "empty"
    START = "start"
    GOAL = "goal"
    OBSTACLE = "obstacle"
    OPEN = "open"
    CLOSED = "closed"

    def __init__(self, row, col, cell_size):
        self.row = row
        self.col = col
        self.x = col * cell_size
        self.y = row * cell_size
        self.cell_size = cell_size
        self.set_type(self.EMPTY)

    def set_type(self, type):
        """ Set the type of cell and its respective color """
        self.type = type
        match self.type:
            case self.START:
                self.color = ORANGE
            case self.GOAL:
                self.color = TURQUOISE
            case self.OBSTACLE:
                self.color = BLACK
            case self.OPEN:
                self.color = GREEN
            case self.CLOSED:
                self.color = RED
            case _:
                self.color = WHITE

    def draw(self, window):
        """ Display the cell as a rectangle in the map with its correspondive color and location """
        pygame.draw.rect(window, self.color, (self.x, self.y, self.cell_size, self.cell_size))

def init_map(rows, cols, cell_size):
    """ Return a 2D array of size rows x cols """
    map = []
    for i in range(rows):
        map.append([])
        for j in range(cols):
            cell = Cell(i, j, cell_size)
            map[i].append(cell)

    return map

def draw_cells(window, map):
    """ Draws rectangles of different colors for each cell based on their state """
    for row in map:
        for cell in row:
            cell.draw(window)

def draw_grid(window, rows, cols, cell_size):
    """ Draws horizontal and vertical lines to form a grid """
    for i in range(rows):
        pygame.draw.line(window, GREY, (0, i*cell_size), (cols*cell_size, i*cell_size))

    for i in range(cols):
        pygame.draw.line(window, GREY, (i*cell_size, 0), (i*cell_size, rows*cell_size))

def draw(window, rows, cols, cell_size, map):
    """ Visualize the state of the map on the pygame window """

    # Start with a white empty screen
    window.fill(WHITE)

    # Draw all cells in the map
    draw_cells(window, map)

    # Visualize the grid
    draw_grid(window, rows, cols, cell_size)

    pygame.display.update()


def main():

    # Create a pygame window
    window = pygame.display.set_mode((SCREEN_LENGTH, SCREEN_LENGTH))
    pygame.display.set_caption("Motion Planning")

    # Initialize the map
    cell_size = SCREEN_LENGTH // CELLS_PER_DIMENSION
    map = init_map(CELLS_PER_DIMENSION, CELLS_PER_DIMENSION, cell_size)

    run = True
    while run:

        # Visualize everything
        draw(window, CELLS_PER_DIMENSION, CELLS_PER_DIMENSION, cell_size, map)

        # Handle user events
        for event in pygame.event.get():

            # User clicked on close button
            if event.type == pygame.QUIT:
                run = False

    # Finish execution
    pygame.quit()


if __name__ == "__main__":
    main()
