import numpy as np
from geom_utils import *

class Circle:
    def __init__(self, x, y, r):
        self.set_center(x, y)
        self.set_radius(r)

    def get_center(self):
        return np.array([self.x, self.y])

    def get_radius(self):
        return self.r

    def set_center(self, x, y):
        self.x = x
        self.y = y

    def set_radius(self, r):
        self.r = r

    def compute_distance(self, other_circle):
        c1 = self.get_center()
        c2 = other_circle.get_center()
        r2 = other_circle.get_radius()
        return distance(c1,c2) - self.r - r2

    def __str__(self):
        return "[("+str(self.x) + ", " + str(self.y) + "), " + str(self.r) + "]"

if __name__ == "__main__":
    c = Circle(3,0,5)
    c2 = Circle(x=3,y=7,r=1)
    print(c, c.compute_distance(c2))
