import math, sys, os
import csv
import numpy as np
import rotate as ro
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

"""
Assumes an aligned point cloud where z+ points upwards from the floor
"""




if __name__ == "__main__":
    res = 4
    bins = (res, res)
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
    # normalize to percentages
    heatmap = heatmap / np.sum(heatmap)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    img = ax.imshow(heatmap, aspect=0.7)#, cmap='cool')
    plt.axis('off')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    plt.colorbar(img, cax=cax)
    ax.axes.set_aspect('auto')
    plt.tight_layout()
    plt.subplots_adjust(left=0.01, right=0.9, top=0.99, bottom=0.03)
    plt.savefig(outPath)
