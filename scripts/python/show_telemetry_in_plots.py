import os
import pickle
import time

import numpy as np
import matplotlib.pyplot as plt

def main():
    folder_path = "../../data/telemetry/"

    files = os.listdir(folder_path)
    for file in files:
        with open(folder_path + file, "rb") as f:
            errors = pickle.load(f)

        timestamp = file.split("-")[0].split(".")[0]
        name = file.split("-")[1].split(".")[0].replace("_", " ")

        datetime = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(int(timestamp)))

        if name == "frame times":
            name = "FPS"
            errors = np.reciprocal(errors)

        plt.plot(errors)
        plt.xlabel("Frame")
        plt.ylabel(name)
        plt.title(f"{name} ({datetime})")
        plt.show()


if __name__ == "__main__":
    main()