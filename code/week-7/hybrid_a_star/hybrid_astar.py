import numpy as np
import math

class HybridAStar:
    # Determine how many grid cells to have for theta-axis.
    NUM_THETA_CELLS = 180

    # Define min, max, and resolution of steering angles
    omega_min = -35
    omega_max = 35
    omega_step = 5

    # A very simple bicycle model
    speed = 1.0
    length = 0.5

    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)

    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []

        # Consider a discrete selection of steering angles.
        for delta_t in np.arange(self.omega_min, self.omega_max, self.omega_step):
            # TODO: implement the trajectory generation based on

            # a simple bicycle model.
            omega = self.speed / self.length * math.tan(math.radians(delta_t))
            next_x = x + (self.speed * math.cos(theta))
            next_y = y + (self.speed * math.sin(theta))

            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            next_theta = theta + omega

            if next_theta == 2 * np.pi :
                next_theta == 0.0

            while next_theta > 2 * np.pi:
                next_theta = next_theta - 2 * np.pi
            while next_theta < 0.0:
                next_theta = 2 * np.pi + next_theta

            # The f value of a newly expanded cell increases by heuristic value
            # from the g value of a newly expanded cell
            f2 = g2 + self.heuristic(next_x, next_y, goal)

            next_s = {
                'f': f2,
                'g': g2,
                'x': next_x,
                'y': next_y,
                't': next_theta,
            }
            next_states.append(next_s)
            # Check validity and then add to the next_states list.
            # Check validity will be in search function

        return next_states

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            # print(self.idx(x), self.idx(y), math.degrees(curr['t']))
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                # Check validity and then add to the next_states list.
                x_ = self.idx(n['x'])
                y_ = self.idx(n['y'])
                stack_ = self.theta_to_stack_num(n['t'])

                if 0 <= x_ < grid.shape[0] \
                    and 0 <= y_ < grid.shape[1] \
                    and grid[(x_, y_)] == 0 \
                    and self.closed[stack_][x_][y_] == 0 :

                    opened.append(n)

                    self.closed[stack_][x_][y_] = 1
                    self.came_from[stack_][x_][y_] = curr

                    total_closed += 1

                # pass
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        partition = np.linspace(0, math.radians(360), self.NUM_THETA_CELLS)
        for i in range(self.NUM_THETA_CELLS - 1) :
            if partition[i] <= theta and theta < partition[i+1]:
                stack_num = i
                break

            else :
                stack_num = self.NUM_THETA_CELLS - 1

        return stack_num

    # Calculate the index of the grid cell based on the vehicle's position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        # dist = (math.sqrt((x-goal[0])**2 + (y-goal[1])**2))
        dist = abs(x - goal[0]) + abs(y - goal[1])
        return int(np.floor(dist))

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path

    def test(self, grid, goal):
        heuri = np.zeros(grid.shape)
        for x in range(grid.shape[0]) :
            for y in range(grid.shape[1]) :
                heuri[(x, y)] = self.heuristic(x, y, goal)

        return heuri


# Test inference
if __name__ == "__main__" :
    has = HybridAStar((90, 16, 16))
    # print(has.theta_to_stack_num(math.radians(180)))
    grid = np.array([
        [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0],
        [0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
        [0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ], dtype=np.int)
    start = (0.0, 0.0, 0.0)
    goal = (grid.shape[0] - 1, grid.shape[1] - 1)

    print(has.test(grid, goal))
    #
    # success, expanded = has.search(grid, start, goal)