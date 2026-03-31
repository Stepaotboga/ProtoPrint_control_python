import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_map(filename):

    data = np.loadtxt(filename)

    xs = np.unique(data[:,0])
    ys = np.unique(data[:,1])

    grid = data[:,2].reshape(len(xs),len(ys))

    return xs,ys,grid


def show_heightmap(filename="map_2.txt"):

    xs,ys,grid = load_map(filename)

    X,Y = np.meshgrid(xs,ys,indexing='ij')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(X,Y,grid,cmap="viridis")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Height Z")

    plt.title("PCB Height Map")

    plt.show()


if __name__ == "__main__":
    show_heightmap("map_2.txt")