"""
Goes over a directory, looks for all matching filenames (csv's), picks one row and writes them to another file.
"""
import re, sys, os
import polygon2cog as p2c
import validation as v

if __name__ == "__main__":
    # line index we want to collect
    TARGET_LINE = 0
    # priority list of file keys, the first one found is taken
    file_keys = ["cluster_cogs", "controlPoints"]
    regex_str = "_(?P<frameIndex>\d+)\.csv"
    if len(sys.argv) >= 3:
        src_dir = os.path.abspath(sys.argv[1])
        out_dir = os.path.abspath(sys.argv[2])
    
    if len(sys.argv) < 3 or not os.path.isdir(src_dir):
        print('Usage: \n\
                Argument 1: source directory with files to aggregate\n\
                Argument 2: output directory')
    frame_indices = [[] for i in range(len(file_keys))]
    file_key = "notFound"
    # search for file key
    for i in range(len(file_keys)):
        file_key_t = file_keys[i]
        frame_indices[i] = p2c.findFrameIndices(re.compile(file_key_t + regex_str), 'frameIndex', src_dir)
        file_key = file_keys[i]
    # choose the first non-empty one
    for i in range(len(frame_indices)):
        if len(frame_indices[i]) > 0:
            frame_indices = frame_indices[i]
            file_key = file_keys[i]
            break
    frame_indices = sorted(frame_indices)
    agg = []
    for f in frame_indices:
        path = os.path.join(src_dir, file_key + "_" + str(f) + ".csv")
        lines = v.read_controlpoints(path)
        row = lines[TARGET_LINE].T.tolist()[0]
        row.insert(0, f)
        agg.append(row) 
    print(agg)
    if not os.path.isdir(out_dir):
        os.mkdir(out_dir)
    out_file = os.path.join(out_dir, 'aggregated.csv')
    p2c.write_csv(out_file, agg)
    



