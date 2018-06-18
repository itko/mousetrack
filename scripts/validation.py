# -*- coding: utf-8 -*-
"""
Created on Tue May 15 19:30:13 2018

@author: Luzian

USAGE:
    1st Argument: directory of controlPoints_*.csv
    2nd Argument: directory of pic*.png, params*.csv files
    3rd Argument: directory of output images
"""
import sys,os,csv,cv2
import numpy as np

# Camera parameters
CAMCHAIN_MAT_DIM = (4,4)
N_STREAMS = 4
X_SHIFT = 22
Y_SHIFT = -8
PREVIEW = 0

# Visualization parameters
DRAW_COG = 1
DRAW_CONTROLPOINTS = 0
DRAW_TRAJECTORY = 0
TRAJECTORY_LENGTH = 20
DRAW_ORIENTATION = 0

# Set to 1 if the .csv's were generated before 21. May 2018
OLD_COG_READ_TYPE = 1

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

def read_controlpoints(csv_path):
    csv_path = os.path.abspath(csv_path)
    points = []
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file)
        for cp in csv_reader:
            points.append(np.asarray([cp],dtype='float64').T)
    return points

def make_projection_matrix(camchain,R,cam):
    matrix = np.identity(4)
    for i in range(2*cam + 1):
        matrix = np.matmul(camchain[i],matrix)
    matrix = np.matmul(R[cam],matrix)
    return np.asarray(matrix,dtype='float64')

def project_point(control_point, projection_matrix, params):
    """
    Takes a 3 dimensional control point (np.array([[x,y,z]])), a 4x4 projection_matrix from world to camera, and 
    params = [focallength, _, ccx, ccy]
    """
    focallength = float(params[0])
    ccx = float(params[2])
    ccy = float(params[3])
    cp_hom = np.row_stack((control_point,[[1]]))
    p4D = np.matmul(projection_matrix , cp_hom)

    p2D = np.array([p4D[0]/p4D[2]*focallength, p4D[1]/p4D[2]*focallength])
    cc = np.array([[ccx, ccy]]).T
    shift = np.array([[X_SHIFT,Y_SHIFT]]).T
    return p2D - shift + cc

colors = [
        (0, 0, 255), 
        (0, 255, 0), 
        (255, 0, 0), 
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 255, 255),
        (0,0,0)
        ]

def point_color(i):
    return colors[i % len(colors)]
    

if __name__ == "__main__":
    # choose your data source files, cluster_cogs work on a per frame basis
    file_keys = ['cluster_cogs', 'controlPoints']
    #Parse Arguments
    if len(sys.argv) < 4 or not os.path.isdir(os.path.abspath(sys.argv[1])) or not os.path.isdir(os.path.abspath(sys.argv[2])):

        print('USAGE: \n\
                1st Argument: directory of controlPoints_*.csv\n\
                2nd Argument: directory of pic*.png, params*.csv files\n\
                3rd Argument: directory of output images')
        sys.exit(1)
    csv_dir = os.path.abspath(sys.argv[1])
    png_dir = os.path.abspath(sys.argv[2])
    out_dir = os.path.abspath(sys.argv[3])


    if os.path.isfile(os.path.join(png_dir,'params_camchain.csv')):
        camchain_path = os.path.join(png_dir,'params_camchain.csv')
        camchain = read_matrix_chain(camchain_path)
    else:
        print('ERROR: params_camchain.csv not found. Terminating...')
        sys.exit(1)

    if os.path.isfile(os.path.join(png_dir,'params_R.csv')):
        R_path = os.path.join(png_dir,'params_R.csv')
        R = read_matrix_chain(R_path)
    else:
        print('ERROR: params_R.csv not found. Terminating...')
        sys.exit(1)

    if not os.path.isdir(out_dir):
        os.mkdir(out_dir)
    #Find all valid controlPoints.csv
    csv_files = os.listdir(csv_dir)
    frame_indices = [[] for i in range(len(file_keys))]
    for file in csv_files:
        # If we find a valid controlPoints*.csv, we extract the index and add it to the list of frames we want to validate.
        # Otherwise, we remove the file from the csv_files list.
        for f in range(len(file_keys)):
            key_str = file_keys[f]
            if file[:len(key_str)+1] == key_str + '_' and file[-4:] == '.csv':
                frame_indices[f].append(int(file[len(key_str)+1:-4]))
    
    # Choose first non empty one
    for i in range(len(frame_indices)):
        if len(frame_indices[i]) > 0:
            frame_indices = frame_indices[i]
            key_str = file_keys[i]
            break
    if DRAW_TRAJECTORY:
        trajectory = []
    frame_indices.sort()
    for frame in frame_indices:
        if frame % 100 == 0:
            print("Processing frame " + str(frame))
        param_file = os.path.join(png_dir,"params_f_" + str(frame) + ".csv")
        if not os.path.isfile(param_file):
            print("Parameter .csv not found - skipping frame " + str(frame))
            continue
        
        if DRAW_CONTROLPOINTS or DRAW_ORIENTATION:
            cpoints = read_controlpoints(os.path.join(csv_dir,key_str + "_" + str(frame) + ".csv"))
            
        if DRAW_COG:
            if OLD_COG_READ_TYPE:
                name = key_str + "_" + str(frame) + ".csv"
            else:
                name = "raw_point_cloud_metrics_" + str(frame) + ".csv"
            cog = read_controlpoints(os.path.join(csv_dir,name))
            cog = cog[0]
            
        if DRAW_TRAJECTORY:
            trajectory.append(cog)
            while len(trajectory) > TRAJECTORY_LENGTH + 1:
                trajectory.pop(0)
                
        if DRAW_ORIENTATION:
            head = []
            for point in cpoints:
                if point.any():
                    head = point
                    break
            
        params = read_params(param_file)
        pics = []
        
        for stream in range(N_STREAMS):
            png_file = os.path.join(png_dir,"pic_s_" + str(stream+1) + "_f_" + str(frame) + ".png")
            
            if not os.path.isfile(png_file):
                print("PNG for frame " + str(frame) + " stream " + str(stream) + " not found. Skipping.")
                continue
            pic = cv2.imread(png_file)
            projection_matrix = make_projection_matrix(camchain,R,stream)
            
            if DRAW_CONTROLPOINTS:    
                for ci in range(len(cpoints)): 
                    cp4 = cpoints[ci]
                    cp2 = project_point(cp4,projection_matrix,params[stream])
                    cp2 = cp2.astype(np.int32)
                    #Display control point in image
                    cv2.circle(pic,(cp2[0],cp2[1]),5,point_color(ci),-1)
                    
            if DRAW_COG or DRAW_ORIENTATION:
                cog2D = project_point(cog,projection_matrix,params[stream])
                cog2D = cog2D.astype(np.int32)
                if DRAW_COG:
                    cv2.circle(pic,(cog2D[0],cog2D[1]),5,(0,0,255),-1)
            if DRAW_TRAJECTORY and len(trajectory) >= 2:
                for i in range(len(trajectory) - 1):
                    p1 = trajectory[i]
                    p2 = trajectory[i+1]
                    p1_2D = project_point(p1,projection_matrix,params[stream])
                    p2_2D = project_point(p2,projection_matrix,params[stream])
                    
                    cv2.circle(pic,(p1_2D[0],p1_2D[1]),3,(0,0,255),-1)
                    cv2.line(pic,(p1_2D[0],p1_2D[1]),(p2_2D[0],p2_2D[1]),(0,0,255),2)
            if DRAW_ORIENTATION and len(head) > 0:
                head2D = project_point(head,projection_matrix,params[stream])
                orientation = head2D - cog2D
                orientation = orientation / np.linalg.norm(orientation) * 50 #50 is length of orientation indicator
                endpoint = cog2D + orientation
                cv2.arrowedLine(pic,(cog2D[0],cog2D[1]),(endpoint[0],endpoint[1]),(0,255,0),2)
                    
                
            pics.append(pic)
            
        pics12 = np.hstack((pics[0],pics[1]))
        pics34 = np.hstack((pics[2],pics[3]))
        pic = np.vstack((pics12,pics34))
        
        if PREVIEW:
            cv2.destroyAllWindows()
            cv2.imshow('Image',pic)
            cv2.moveWindow('Image',0,0)
            key = cv2.waitKey(0)
            if key == 27 or key == 113:
                exit(0)
        cv2.imwrite(os.path.join(out_dir,'val_f_' + str(frame) +'.png'),pic)
    cv2.destroyAllWindows()
