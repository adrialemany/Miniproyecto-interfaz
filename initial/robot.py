from object import Object

class Robot(Object):
    def __init__(self, x, y, size, name=None):
        super().__init__(x, y, size, name)

    def move(self, direction, speed):
        current_pos = super().get_position()
        target_pos = current_pos+speed*direction
        super().set_position(target_pos[0], target_pos[1])

    def __str__(self):
        return "Robot. " + super().__str__()


if __name__ == "__main__":
    rob1 = Robot(10, 10, 2, "R1")
    rob2 = Robot(14, 10, 2, "R2")
    print("robot: ", rob1)
    print("distance between "+str(rob1)+ " and " + str(rob2) + " is "+str(rob1.compute_distance(rob2)))