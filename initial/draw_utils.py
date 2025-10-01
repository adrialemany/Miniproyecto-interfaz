import matplotlib.pylab as plt

def draw_path(path):
    plt.plot(path[:,0],path[:,1],'o')
    #plt.plot(path)
    plt.show(block=True)