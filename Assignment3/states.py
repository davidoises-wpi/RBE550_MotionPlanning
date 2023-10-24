from abc import ABC, abstractmethod
from math import tan, cos, sin, pi, radians, degrees, sqrt

class State(ABC):

    def __init__(self, x, y, angle):
        super().__init__()
        self.x = x
        self.y = y
        self.angle = angle
        self.parent_state = None
        self.cost = 0
        self.total_cost = 0

    @abstractmethod
    def get_neighbors(self, vehicle, obstacles, dt_horizon, goal_state):
        pass

    def calculate_euclidean_distance(self, other_state):
        """ Euclidean norm distance """

        x_diff = self.x - other_state.x
        y_diff = self.y - other_state.y

        dist = sqrt(x_diff**2 + y_diff**2)

        return dist

    def __eq__(self, other_state):
        """ Compares 2 states considering certain tolerance """
        dist = self.calculate_euclidean_distance(other_state)
        angle_diff = degrees(abs(self.angle - other_state.angle)%(2*pi))

        return dist < 10 and angle_diff < 5

class AckermannDriveState(State):

    # 25 meters per second max speed
    MAX_V_M = 25

    # Max wheel angle of 45 degrees
    MAX_PSI_DEG = 45

    WHEELBASE_M = 2.8

    WHEELBASE_PIXELS = 86

    def __init__(self, x, y, angle, psi, v, dt):
        super().__init__(x, y, angle)
        self.psi = psi
        self.dt = dt
        self.v = v

    def get_steps(self, dt, psi_rad, v, current_angle):
        """ This function returns the steps after integrating the kinematic model over dt """
        omega = (v/self.WHEELBASE_M)*tan(psi_rad)
        dangle = omega*dt
        angle = (current_angle + dangle)%(2*pi)

        dx = v*cos(angle)*dt*self.WHEELBASE_PIXELS/self.WHEELBASE_M
        dy = -v*sin(angle)*dt*self.WHEELBASE_PIXELS/self.WHEELBASE_M # invert y due to pixel orientation

        return dangle, dx, dy

    def get_neighbors(self, vehicle, obstacles, dt_horizon, open_states, closed_states, goal_state, xlimit, ylimit):
        """ This function expands the reachable states from the current state given the kinematic constraints
            and calculates the cost to reach the new states using the path cost and a heuristic cost
        """
        neighbors = []

        psi_increment_deg = 5
        for v in [-self.MAX_V_M, self.MAX_V_M]:
            for psi_deg in range(-self.MAX_PSI_DEG, self.MAX_PSI_DEG+psi_increment_deg, psi_increment_deg):
                psi_rad = radians(psi_deg)
                dangle, dx, dy = self.get_steps(dt_horizon, psi_rad, v, self.angle)

                angle = (self.angle + dangle)%(2*pi)

                x = round(self.x + dx)
                y = round(self.y + dy)

                original_x = vehicle.x
                original_y = vehicle.y
                original_angle = vehicle.orientation

                # 1. Check if its withing map limits
                if x >= xlimit[0] and x <= xlimit[1] and y >= ylimit[0] and y <= ylimit[1]:

                    # 2. Check if the state hasnt been explored yet
                    new_state = AckermannDriveState(x, y, angle, psi_rad, v, dt_horizon)
                    if new_state not in closed_states:

                        # 3. Check if the new state is in collision
                        vehicle.set_position(x, y)
                        vehicle.set_orientation(degrees(angle))
                        vehicle.update()
                        colided = vehicle.check_collision(obstacles)
                        if not colided:

                            # Calculate the new costs
                            tentative_new_cost = self.cost + self.calculate_euclidean_distance(new_state)
                            heuristic_cost = new_state.calculate_heuristic_cost(self, goal_state)

                            # Update the state in the open list if it exists
                            try:
                                index = open_states.index(new_state)
                                previous_cost = open_states[index].cost
                                if tentative_new_cost < previous_cost:
                                    open_states[index] = new_state
                                    open_states[index].cost = tentative_new_cost
                                    open_states[index].total_cost = tentative_new_cost + heuristic_cost
                                    open_states[index].parent_state = self
                            except ValueError:
                                # return a new element in the neighbors list if this hadnt been discovered
                                new_state.parent_state = self
                                new_state.cost = tentative_new_cost
                                new_state.total_cost = tentative_new_cost + heuristic_cost
                                neighbors.append(new_state)
                        vehicle.set_position(original_x, original_y)
                        vehicle.set_orientation(original_angle)
                        vehicle.update()

        return neighbors

    def calculate_heuristic_cost(self, prev_state, goal_state):
        """ Heuristic cost calculation penalizing:
        euclidean distance from goal
        driving in reverse
        agressive changes of steering
        the difference between the current orientation of the vehicle and the goal orientation
        """

        distance_multiplier = 2
        reverse_multiplier = 1
        if self.v < 0:
            reverse_multiplier = 1.8
        steering_multiplier = 1
        if (self.v * prev_state.v) < 0:
            steering_multiplier = 1.5
        dist = self.calculate_euclidean_distance(goal_state)
        angle_diff = degrees(abs(self.angle-goal_state.angle)%(2*pi))

        return steering_multiplier*reverse_multiplier*distance_multiplier*dist + angle_diff

    def goal_check(self, other_state):
        """ Checks if the current state is whithin acceptable tolerance from the goal """
        dist = self.calculate_euclidean_distance(other_state)
        angle_diff = degrees(abs(self.angle - other_state.angle)%(2*pi))

        return dist <10 and angle_diff < 5

    def __str__(self):
        return f"x: {self.x} y: {self.y} angle: {degrees(self.angle)} psi: {degrees(self.psi)} v: {self.v} cost: {self.cost}"







