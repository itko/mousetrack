import math, sys, os
import numpy as np

from plyfile import PlyData

def rotX(alpha):
    Rxc = math.cos(alpha)
    Rxs = math.sin(alpha)
    Rx = np.array([[1, 0, 0], [0, Rxc, -Rxs], [0, Rxs, Rxc]])
    return Rx

def rotY(beta):
    Ryc = math.cos(beta)
    Rys = math.sin(beta)
    Ry = np.array([[Ryc, 0, Rys], [0, 1, 0], [-Rys, 0, Ryc]])
    return Ry

def rotZ(gamma):
    Rzc = math.cos(gamma)
    Rzs = math.sin(gamma)
    Rz = np.array([[Rzc, -Rzs, 0], [Rzs, Rzc, 0], [0, 0, 1]])
    return Rz


def rotationMatrix(alpha, beta, gamma):
    Rx = rotX(alpha)
    Ry = rotY(beta)
    Rz = rotZ(gamma)
    return np.matmul(Rx, np.matmul(Ry, Rz))

def readFile(path):
    if path[-3:] == 'ply':
        return readPly(path)
    print("Warning: could not read " + str(path))
    return (None, None)
def writeFile(path, verts, fObj):
    if path[-3:] == 'ply':
        return writePly(path, verts, fObj)
    print("Warning, could not write to " + str(path))

def readPly(path):
    """
    reads vertex data into a 3xn matrix, returns a tuple, where the first element is the matrix, and the second element the entire file object
    """
    plydata = PlyData.read(path)
    verts = plydata["vertex"]
    vertData = np.array([verts["x"], verts["y"], verts["z"]])
    return (vertData, plydata)

def writePly(path, vertData, plydata):
    """
    pluggs vertData into plydata and writes it to path
    """
    plydata["vertex"]["x"] = vertData[0]
    plydata["vertex"]["y"] = vertData[1]
    plydata["vertex"]["z"] = vertData[2]
    plydata.write(path)

if __name__ == "__main__":
    alphaX = (180+48.0)/360*2*math.pi
    alphaY = 3/360*2*math.pi
    alphaZ = 0
    if len(sys.argv) >= 3:
        srcPath = os.path.abspath(sys.argv[1])
        outPath = os.path.abspath(sys.argv[2])
    if len(sys.argv) < 3 or not os.path.isfile(srcPath):
        print('Usage: \n\
                Argument 1: Source path to file with points\n\
                Argument 2: Target path where new file should be written')
        sys.exit(1)
    rotMat = rotationMatrix(alphaX, alphaY, alphaZ)
    print("Reading...")
    (verts, fObj) = readFile(srcPath)
    print("Transforming...")
    verts = np.matmul(rotMat, verts)
    print("Writing...")
    writeFile(outPath, verts, fObj)
