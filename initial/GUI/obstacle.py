from object import Object

class Obstacle(Object):
    def __init__(self, x, y, size, name=None):
        super().__init__(x, y, size, name)

    def __str__(self):
        return "Obstacle. " + super().__str__()

if __name__ == "__main__":
    obs = Obstacle(10, 10, 2, "Obs1")
    obs2 = Obstacle(14, 10, 2, "Obs2")
    print("obst: ", obs)
    print("distance between "+str(obs)+ " and " + str(obs2) + " is "+str(obs.compute_distance(obs2)))