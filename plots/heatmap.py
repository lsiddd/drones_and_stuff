import time

from numba import jit
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(10000)


def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.2f ms' %
                  (method.__name__, (te - ts) * 1000))
        return result

    return timed


@timeit
def create_sources(N=5, range=1000):
    # create N 2 dimentional tuples whic correspond to random 2d locations in the scenario
    return np.random.randint(range, size=(N, 2))


def thr_function(x, max=10, decay=100):
    return max / np.exp(x / decay)


@timeit
def populate(dimensions, sources, max=10, decay=100):
    # scenario = np.array(size=(dimensions, dimensions))
    scenario = np.zeros(shape=(dimensions, dimensions), dtype=np.float64)
    for x in range(dimensions):
        for y in range(dimensions):
            coord = np.array([x, y])
            dist = get_distance_to_closest_source(coord, sources)
            scenario[x][y] = thr_function(dist, max=max, decay=decay)
    return scenario


@jit(nopython=True)
def get_distance_to_closest_source(coord, sources):
    # receive a 2d coordinate and a list of 2d coordinates and pick the closest coord from the list
    dists = np.zeros(len(sources), dtype=np.float64)
    for index, s in enumerate(sources):
        dists[index] = np.sqrt((coord[0] - s[0]) ** 2 + (coord[1] - s[1]) ** 2)
        # dists[index] = np.linalg.norm(coord-s)
    return min(dists)


def show(scenario, sources=None, save=False, filename="heatmap.pdf",
         label="Available Throughput (Mbps)", cmap="viridis", sources_label="Base Stations"):
    plt.figure(figsize=(12, 12))
    plt.imshow(scenario, cmap=cmap)
    if(sources is not None):
        plt.scatter(sources[:, 1], sources[:, 0], marker="^", c="red", label=sources_label)

    plt.xticks(fontsize=22)
    plt.yticks(fontsize=22)
    plt.xlabel("X Coordinate", fontsize=22)
    plt.ylabel("Y Coordinate", fontsize=22)

    plt.legend(fontsize=22)

    cbar = plt.colorbar()
    # cbar.ax.set_ylabel(label, rotation=270, fontsize=22)
    cbar.ax.text(1.5, 0.5, label, ha='center', va='center', rotation=270,
                 transform=cbar.ax.transAxes, fontsize=22)
    cbar.ax.yaxis.set_ticks_position('left')

    if (save):
        plt.savefig(filename, bbox_inches="tight")
    else:
        plt.show()


def main():
    dims = 2000
    sources = create_sources(20, dims)
    scenario = populate(dims, sources, max=10, decay=500)
    show(scenario, sources, save=True, filename="coverage.pdf")

    request_sources = create_sources(7, dims)
    request_density = populate(dims, request_sources, 5, decay=200)
    show(request_density, save=True, filename="requested.pdf",
         label="Downlink Usage (Mbps)", cmap='inferno')
    show(scenario - request_density, save=True, filename="difference.pdf",
         label="Downlink Capacit Deficit (Mbps)", cmap='magma')
    show(request_density - scenario, save=True, filename="difference2.pdf",
         label="Downlink Capacit Deficit (Mbps)", cmap='magma')


if __name__ == "__main__":
    main()
