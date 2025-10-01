from circle import Circle

class Object:
    def __init__(self,x, y, size, name=None):
        self.circle = Circle(x, y, size)
        self.name = name

    def set_position(self, x, y):
        self.circle.set_center(x, y)

    def set_size(self, size):
        self.circle.set_radius(size)

    def get_position(self):
        return self.circle.get_center()

    def get_size(self):
        return self.circle.get_radius()

    def __str__(self):
        return str(self.name) + ": " + str(self.circle)

    def compute_distance(self, other_object):
        return self.circle.compute_distance(other_object.circle)


if __name__ == "__main__":
    o = Object(5, 3, 2)
    d = 5
    r2 = 5
    o2 = Object(o.get_position()[0]+o.get_size()+d+r2, o.get_position()[1], r2)
    print(o,o2)
    assert o.compute_distance(o2)==d, "Check distance computation"
    assert o.compute_distance(o2)==o2.compute_distance(o), "Check distance computation"
