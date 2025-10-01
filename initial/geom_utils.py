import numpy as np

def point_to_str(p):
    return "(" + str(p[0]) + ", " + str(p[1]) + ")"

def module(p):
    return np.linalg.norm(p)

def direction_vector(p1, p2):
    return p2-p1

def distance(p1, p2):
    dir_vec = direction_vector(p1, p2)
    return module(dir_vec)

def unit_vector(v):
    dist = module(v)
    return v/dist

if __name__ == "__main__":
    c1 = np.array([0,1])
    d=5
    c2 = c1+np.array([0,d])
    #print("module of c1", module(c1))
    assert distance(c1,c2)==d, "check distances"
