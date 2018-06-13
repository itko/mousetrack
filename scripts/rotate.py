import math, sys, os
import csv
import numpy as np

from plyfile import PlyData, PlyElement

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




def readFile(path):
    if path[-3:] == 'ply':
        return readPly(path)
    if path[-3:] == 'csv':
        return readCsv(path)
    print("Warning: could not read " + str(path))
    return (None, None)
def writeFile(path, verts, fObj):
    if path[-3:] == 'ply':
        return writePly(path, verts, fObj)
    if path[-3:] == 'csv':
        return writeCsv(path, verts, fObj)
    print("Warning, could not write to " + str(path))

def readPly(path):
    """
    reads vertex data into a 3xn matrix, returns a tuple, where the first element is the matrix, and the second element the entire file object
    """
    plydata = PlyData.read(path)
    verts = plydata["vertex"]
    vertData = np.array([verts["x"], verts["y"], verts["z"]])
    return (vertData, (plydata, 'ply'))

def writePly(path, vertData, plydata):
    """
    pluggs vertData into plydata and writes it to path
    """
    if plydata[1] == 'ply':
        plydata = plydata[0]
        plydata["vertex"]["x"] = vertData[0]
        plydata["vertex"]["y"] = vertData[1]
        plydata["vertex"]["z"] = vertData[2]
    else:
        mapped = list(map(lambda x : tuple(x), vertData.T.tolist()))
        vertex = np.array(mapped,
                   dtype=[('x', 'f4'), ('y', 'f4'),
                          ('z', 'f4')])
        el = PlyElement.describe(vertex, 'vertex')
        plydata = PlyData([el])
    plydata.write(path)

def readCsv(csv_path):
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file)
        rows = []
        for r in csv_reader:
            rows.append(r)
        csvdata = np.array(rows)
        verts = [csvdata[:,1], csvdata[:,2], csvdata[:,3]]
        verts = np.array(verts)
        verts = verts.astype(float)
        return (verts, (csvdata, 'csv'))

def writeCsv(csv_path, verts, fObj):
    if fObj[1] == 'csv':
        fObj = fObj[0]
        fObj[:,1] = verts[0]
        fObj[:,2] = verts[1]
        fObj[:,3] = verts[2]
    else:
        fObj = verts.T
    with open(csv_path, 'w+') as csv_file:
        csv_writer = csv.writer(csv_file)
        for row in fObj:
            csv_writer.writerow(row)



if __name__ == "__main__":
    alphaX = (180+45.0)/360*2*math.pi
    betaY = 3/360*2*math.pi
    gammaZ = -20/360*2*math.pi
    Rx = rotX(alphaX)
    Ry = rotY(betaY)
    Rz = rotZ(gammaZ)
    rotMat = np.matmul(Rz, np.matmul(Rx, Ry))
    dt = np.array([0,0,0.285]).T
    if len(sys.argv) >= 3:
        srcPath = os.path.abspath(sys.argv[1])
        outPath = os.path.abspath(sys.argv[2])
    if len(sys.argv) < 3 or not os.path.isfile(srcPath):
        print('Usage: \n\
                Argument 1: Source path to file with points\n\
                Argument 2: Target path where new file should be written')
        sys.exit(1)
    print("Reading...")
    (verts, fObj) = readFile(srcPath)
    print("Transforming...")
    verts = np.matmul(rotMat, verts)
    verts += dt[:,np.newaxis]
    print("Writing...")
    writeFile(outPath, verts, fObj)
