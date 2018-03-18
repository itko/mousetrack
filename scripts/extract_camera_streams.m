close all;
clear;

rosbag_path = '2018-01-23-10-24-43.bag';
camchain_path = 'camchain-2018-01-22-18-51-48.yaml';
output_path = './data/2018-01-23-10-24-43';

display = false; % show processing steps

% how many camera streams are there? (number of disparity maps)
streams = 4;

% create ouput path if it doesn't exist
if 0 == exist(output_path, 'dir')
    mkdir(output_path);
end

%read rosbag
fprintf('Reading bag...\n');
bag = rosbag(rosbag_path);
fprintf('Preparing loop...\n');

%read calibration file
camchain = YAML.read(camchain_path);

%calculate minor rotation correction (R_0-R_3) based on calibration params
undistrect;

T_cn_cnm0 = eye(4);
T_cn_cnm1 = camchain.cam1.T_cn_cnm1;
T_cn_cnm2 = camchain.cam2.T_cn_cnm1;
T_cn_cnm3 = camchain.cam3.T_cn_cnm1;
T_cn_cnm4 = camchain.cam4.T_cn_cnm1;
T_cn_cnm5 = camchain.cam5.T_cn_cnm1;
T_cn_cnm6 = camchain.cam6.T_cn_cnm1;
T_cn_cnm7 = camchain.cam7.T_cn_cnm1;

%extract camera image messages
img_streams(1) = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_rect');
img_streams(2) = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_depth');
img_streams(3) = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_rect');
img_streams(4) = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_depth');
img_streams(5) = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_rect');
img_streams(6) = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_depth');
img_streams(7) = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_rect');
img_streams(8) = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_depth');

%extract camera info messages
bagselect8 = select(bag, 'Topic', '/uvc_baseboard0/cam_0/camera_info');
bagselect10 = select(bag, 'Topic', '/uvc_baseboard0/cam_2/camera_info');
bagselect12 = select(bag, 'Topic', '/uvc_baseboard1/cam_0/camera_info');
bagselect14 = select(bag, 'Topic', '/uvc_baseboard1/cam_2/camera_info');



% find common image range
lowestIndex = min([img_streams.NumMessages]);

% hope the first image exists to preallocate our array
%msg = readMessages(img_streams(1), 1);
%pic1 = readImage(msg{1});
%[h, w] = size(pic1);
%pics = zeros(streams, h, w, 'uint8');
%depths = zeros(streams, h, w, 'uint8');

% preallocate
out_pics = cell(streams, 1);
out_depths = cell(streams, 1);
pics = cell(streams, 1);
depths = cell(streams, 1);

for image_index = 1:lowestIndex
    progress = ['Extracting: ' int2str(image_index) '/' int2str(lowestIndex) ];
    fprintf(progress);
    
    %Msgs8 = readMessages(bagselect8,image_index);
    %Msgs10 = readMessages(bagselect10,image_index);
    %Msgs12 = readMessages(bagselect12,image_index);
    %Msgs14 = readMessages(bagselect14,image_index);
    
    for i = 1:streams
        out_pics{i} = [output_path '/pic' '_s_' int2str(i) '_f_' int2str(image_index) '.png' ];
        out_depths{i} = [output_path '/depth' '_s_' int2str(i) '_f_' int2str(image_index) '.png' ];
    end
    % some logic to skip frames we've already processed
    skip = true;
    for i = 1:streams
        if ~exist(out_pics{i}, 'file') || ~exist(out_depths{i}, 'file')
            skip = false;
            break;
        end
    end
    if skip
        fprintf(repmat('\b', 1, numel(progress)));
        continue;
    end
    %  core processing
    for i = 1:streams
        msg = readMessages(img_streams(2*i-1), image_index);
        pics{i} = readImage(msg{1});
        msgDepth = readMessages(img_streams(2*i), image_index);
        depths{i} = readImage(msgDepth{1});
        imwrite(pics{i}, out_pics{i});
        imwrite(depths{i}, out_depths{i});
    end
    
    if display
        figure(1)
        %show figure with extracted rect and depth images
        for i = 1:streams
            subplot(2,4,i)
            imshow(pics{i});
            subplot(2,4,i+4)
            imshow(depths{i});
        end
        % give the gui a chance to redraw the figure
        pause(0.1);
    end
    fprintf(repmat('\b', 1, numel(progress)));
end


