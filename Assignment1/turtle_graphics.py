import turtle

def main():

    # Creat a screen and turtle object
    s = turtle.getscreen()
    turtle_searcher = turtle.Turtle()

    # How much the initial angle changes on each search loop
    angle_step = 10

    for i in range(0,3):
        base_angle = i*angle_step
        search_iteration(turtle_searcher, base_angle)

    # Hold the screen 
    turtle_searcher.screen.mainloop()

def search_iteration(t, base_angle):
    # Initial angle
    t.left(base_angle)

    # Calculations for angles are done so that the "angle" variable holds the 
    # absolute angle in relation to the x axis (absolute 0)
    
    # First triangle
    prev_angle = base_angle
    angle = 90
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = -30
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = -150
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)

    # Second triangle
    prev_angle = angle
    angle = -150
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = 90
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = -30
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)

    #Third triangle
    prev_angle = angle
    angle = -30
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = -150
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)
    prev_angle = angle
    angle = 90
    angle += base_angle
    t.left(angle - prev_angle)
    t.forward(100)

if __name__ == "__main__":
    main()