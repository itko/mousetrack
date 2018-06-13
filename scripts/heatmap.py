import math, sys, os
import csv
import numpy as np
import rotate as ro
import matplotlib.pyplot as plt

"""
Assumes an aligned point cloud where z+ points upwards from the floor
"""




if __name__ == "__main__":
    bins = (32, 32)
    if len(sys.argv) >= 3:
        srcPath = os.path.abspath(sys.argv[1])
        outPath = os.path.abspath(sys.argv[2])
    if len(sys.argv) < 3 or not os.path.isfile(srcPath):
        print('Usage: \n\
                Argument 1: Source path to file with aligned points\n\
                Argument 2: Target path where new file should be written')
        sys.exit(1)
    print("Reading...")
    (verts, fObj) = ro.readFile(srcPath)
    heatmap, xedges, yedges = np.histogram2d(verts[0], verts[1], bins=bins)
    plt.imshow(heatmap)
    plt.savefig(outPath)
