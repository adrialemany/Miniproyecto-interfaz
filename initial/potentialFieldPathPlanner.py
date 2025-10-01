from obstacle import Obstacle
from robot import Robot
from geom_utils import *
from draw_utils import *

class PotentialFieldPathPlanner:
    def __init__(self, robot, xG, yG):
        self.set_goal_point(xG,yG)
        self.set_robot(robot)
        self.obstacles = []

        # setting default values
        self.set_max_iterations(20000)
        self.set_influence_area(80)
        self.set_attraction_factor(1)
        self.set_repulsion_factor(20)

    def set_goal_point(self, x, y):
        self.goal = np.array([x, y])

    def set_robot(self, robot):
        self.robot = robot

    def add_obstacle(self, obs):
        self.obstacles.append(obs)

    def _check(self, i, verb):
        n = len(self.obstacles)
        assert 0 <= i < n, "item " + str(i) + " cannot be " + verb + "; list has " + str(n) + " items"

    def remove_obstacle(self, i):
        self._check(i,"removed")
        self.obstacles.pop(i)

    def move_robot(self, x, y):
        self.robot.set_position(x,y)

    def resize_robot(self, size):
        self.robot.set_size(size)

    def move_obstacle(self, i, x, y):
        # move obstacle i-th to new position (x,y)
        self._check(i, "moved")
        self.obstacles[i].set_position(x, y)

    def resize_obstacle(self, i, size):
        # resize obstacle i-th to new size
        self.obstacles[i].set_size(size)

    def __str__(self):
        return str(self.robot) + "\n Goal: " + point_to_str(self.goal)

    def _compute_attractive_foce(self):
        robot_pos = self.robot.get_position()
        d = distance(robot_pos, self.goal)
        f_att = self.attraction_factor * unit_vector(self.goal-robot_pos)
        return f_att, d


    def _compute_repulsive_force(self):
        robot_pos = self.robot.get_position()
        d0 = self.influence_area # influence area (minimum distance to obstacle for it to exert repulsion)
        f_rep = np.array([0,0])
        min_dist = np.inf
        for obs in self.obstacles:
            f_obs = unit_vector(robot_pos-obs.get_position())
            d_obs = distance(robot_pos, obs.get_position())
            min_dist = np.min([min_dist, d_obs])
            if d_obs > d0:
                f_rep_obs = np.zeros(2)
            else:
                f_rep_obs = self.repulsion_factor * (1/d_obs - 1/d0) * f_obs
            f_rep = f_rep + f_rep_obs
        return f_rep, min_dist

    def set_max_iterations(self, n_max):
        self.num_iters_max = n_max

    def set_influence_area(self, d0):
        self.influence_area = d0

    def set_attraction_factor(self, att_factor):
        self.attraction_factor = att_factor

    def set_repulsion_factor(self, rep_factor):
        self.repulsion_factor = rep_factor

    def run(self):
        d_goal = np.inf
        close_enough = 1 # tolerance in distance to goal
        path = []

        iters = 0
        while d_goal>close_enough and iters<self.num_iters_max:
            iters +=1
            robot_pos = self.robot.get_position()
            path.append(robot_pos)
            f_att, d_goal = self._compute_attractive_foce()
            f_rep, d_obs  = self._compute_repulsive_force()
            #print("distance to goal", d_goal)

            #print("f_rep",f_rep)
            #print("f_att",f_att)
            f_total = f_att + f_rep
            d = min(d_goal, d_obs)
            speed = np.exp(d_goal / 100)
            self.robot.move(f_total, speed)
        return np.array(path)


if __name__ == "__main__":
    robot = Robot(0, 0, 5, "turtle")
    pfpp = PotentialFieldPathPlanner(robot, 100, 100)

    pfpp.add_obstacle(Obstacle(20,30,10))
    pfpp.add_obstacle(Obstacle(10,80,25))
    pfpp.add_obstacle(Obstacle(70,60,15))
    pfpp.add_obstacle(Obstacle(80,90,20))
    pfpp.add_obstacle(Obstacle(80,20,20))

    #pfpp.remove_obstacle(1)

    pfpp.move_obstacle(0,20,30)
    pfpp.resize_obstacle(1,0.1)

    print("planner state:\n", pfpp)
    path = pfpp.run()
    draw_path(path)

