# -*- coding: utf-8 -*-
"""
Created on Tue May 15 19:30:13 2018

@author: Luzian

USAGE:
    1st Argument: directory of controlPoints_*.csv
    2nd Argument: directory of input images
    (Optional) 3rd Argument: path to params_camchain.csv [optional, if not set input path is used]
"""
import sys,os,csv,cv2
import numpy as np

CAMCHAIN_MAT_DIM = (4,4)
N_STREAMS = 4
X_SHIFT = 22
Y_SHIFT = -8

def read_matrix_chain(csv_path):
    csv_path = os.path.abspath(csv_path)
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file)
        matrices = []
        for matrix in csv_reader:
            matrices.append(np.asarray(np.reshape(matrix,CAMCHAIN_MAT_DIM).T,dtype='float64'))# Transpose because MATLAB reshape is column wise and numpy is row-wise
        return matrices

def read_params(csv_path):
    csv_path = os.path.abspath(csv_path)
    params = []
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file)
        for entry in csv_reader:
            params.append(entry)
        return params

def read_controlpoint(csv_path):
    csv_path = os.path.abspath(csv_path)
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file)
        point = np.asarray([next(csv_reader)],dtype='float64')
        return point.T

def projection_matrix(camchain,R,cam):
    matrix = R[cam]
    for i in range(2*cam + 1):
        matrix = np.matmul(matrix , camchain[i])
    return np.asarray(matrix,dtype='float64')

def project_point(control_point, projection_matrix, params):
    focallength = float(params[0])
    ccx = float(params[2])
    ccy = float(params[3])
    cp_hom = np.row_stack((control_point,[[1]]))
    p4D = np.matmul(projection_matrix , cp_hom)

    p2D = np.array([p4D[0]/p4D[3]/focallength, p4D[1]/p4D[3]/focallength])
    cc = np.array([[ccx, ccy]]).T
    shift = np.array([[X_SHIFT,Y_SHIFT]]).T
    print(shift.shape)
    return p2D - shift + cc

if __name__ == "__main__":

    #Parse Arguments
    if len(sys.argv) < 3 or not os.path.isdir(os.path.abspath(sys.argv[1])) or not os.path.isdir(os.path.abspath(sys.argv[2])):

        print('USAGE:\n \
              1st Argument: directory of controlPoints_*.csv\n \
              2nd Argument: directory of input images\n')
        sys.exit(1)
    csv_dir = os.path.abspath(sys.argv[1])
    png_dir = os.path.abspath(sys.argv[2])


    if os.path.isfile(os.path.join(png_dir,'params_camchain.csv')):
        camchain_path = os.path.join(png_dir,'params_camchain.csv')
        camchain = read_matrix_chain(camchain_path)
    else:
        print('ERROR: params_camchain.csv not found. Terminating...')
        sys.exit(1)

    if os.path.isfile(os.path.join(png_dir,'params_R.csv')):
        R_path = os.path.join(png_dir,'params_camchain.csv')
        R = read_matrix_chain(R_path)
    else:
        print('ERROR: params_R.csv not found. Terminating...')
        sys.exit(1)


    #Find all valid controlPoints.csv
    csv_files = os.listdir(csv_dir)
    frame_indices = []
    for file in csv_files:
        # If we find a valid controlPoints*.csv, we extract the index and add it to the list of frames we want to validate.
        # Otherwise, we remove the file from the csv_files list.
        if file[:14] == 'controlPoints_' and file[-4:] == '.csv':
            frame_indices.append(int(file[14:-4]))


    for frame in frame_indices:
        param_file = os.path.join(png_dir,"params_f_" + str(frame) + ".csv")
        if not os.path.isfile(param_file):
            print("Parameter .csv not found - skipping frame " + str(frame))
            continue

        params = read_params(param_file)

        for stream in range(N_STREAMS):
            png_file = os.path.join(png_dir,"pic_s_" + str(stream+1) + "_f_" + str(frame) + ".png")
            if not os.path.isfile(png_file):
                print("PNG for frame " + str(frame) + " stream " + str(stream) + " not found. Skipping.")
                continue

            projection_matrix = projection_matrix(camchain,R,stream)
            cp4 = read_controlpoint(os.path.join(csv_dir,"controlPoints_" + str(frame) + ".csv"))
            cp2 = project_point(cp4,projection_matrix,params[stream])
            cp2 = cp2.astype(np.int32)
            pic = cv2.imread(png_file)

            #Display control point in image
            print(cp2)
            cv2.circle(pic,(cp2[0],cp2[1]),5,(0,0,255),-1)
            cv2.imshow(os.path.basename(png_file),pic)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
