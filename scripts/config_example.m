% this is an example for a valid configuration file
% adjust paths according to your needs

% set these paths according to your bag file
rosbag_path = '2018-01-23-10-24-43.bag';
camchain_path = 'camchain-2018-01-22-18-51-48.yaml';

% generated files will be written here
output_path = '../data/2018-01-23-10-24-43';

% do you want to see a live preview?
display = false;

% how many camera streams are there? (number of disparity maps)
streams = 4;

% calculating point clouds takes a moment and might not be necessary
extract_point_clouds = false;
