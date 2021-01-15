from glob import glob

import numpy as np
import matplotlib.pyplot as plt


def main():
    filename = "user_requests/02e8642dd24fc4c4459b686e0e3fa2b8.txt"

    with open(filename) as f:
        lines = f.readlines()
        lines = [float(i.split(",")[7]) * 32 * 1024 for i in lines]
        print(lines)

        plt.style.use("classic")
        plt.grid()
        plt.plot(lines)

        plt.figure()
        plt.boxplot([lines])

        plt.show()


main()
