
%read rosbag
bag = rosbag('2018-01-23-10-24-43.bag');

%read calibration file
camchain = YAML.read('camchain-2018-01-22-18-51-48.yaml');

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
bagselect0 = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_rect');
bagselect1 = select(bag, 'Topic', '/uvc_baseboard0/cam_0/image_depth');
bagselect2 = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_rect');
bagselect3 = select(bag, 'Topic', '/uvc_baseboard0/cam_2/image_depth');
bagselect4 = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_rect');
bagselect5 = select(bag, 'Topic', '/uvc_baseboard1/cam_0/image_depth');
bagselect6 = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_rect');
bagselect7 = select(bag, 'Topic', '/uvc_baseboard1/cam_2/image_depth');

%extract camera info messages
bagselect8 = select(bag, 'Topic', '/uvc_baseboard0/cam_0/camera_info');
bagselect10 = select(bag, 'Topic', '/uvc_baseboard0/cam_2/camera_info');
bagselect12 = select(bag, 'Topic', '/uvc_baseboard1/cam_0/camera_info');
bagselect14 = select(bag, 'Topic', '/uvc_baseboard1/cam_2/camera_info');

%select 500th sample from set
image_index = 500;

Msgs8 = readMessages(bagselect8,image_index);
Msgs10 = readMessages(bagselect10,image_index);
Msgs12 = readMessages(bagselect12,image_index);
Msgs14 = readMessages(bagselect14,image_index);

Msgs0 = readMessages(bagselect0,image_index);
image0 =readImage(Msgs0{1});
Msgs0depth = readMessages(bagselect1,image_index);
image0_depth =readImage(Msgs0depth{1});
Msgs1 = readMessages(bagselect2,image_index);
image1 =readImage(Msgs1{1});
Msgs1depth = readMessages(bagselect3,image_index);
image1_depth =readImage(Msgs1depth{1});

Msgs2 = readMessages(bagselect4,image_index);
image2 =readImage(Msgs2{1});
Msgs2depth = readMessages(bagselect5,image_index);
image2_depth =readImage(Msgs2depth{1});
Msgs3 = readMessages(bagselect6,image_index);
image3 =readImage(Msgs3{1});
Msgs3depth = readMessages(bagselect7,image_index);
image3_depth =readImage(Msgs3depth{1});

%show figure with extracted rect and depth images
figure(1)
subplot(2,4,1)
imshow(image0);
subplot(2,4,5)
imshow(image0_depth);
subplot(2,4,2)
imshow(image1);
subplot(2,4,6)
imshow(image1_depth);
subplot(2,4,3)
imshow(image2);
subplot(2,4,7)
imshow(image2_depth);
subplot(2,4,4)
imshow(image3);
subplot(2,4,8)
imshow(image3_depth);

%initialize 4 point clouds with zeros
xyzPoints0=single(zeros(480,752,3));
xyzPoints1=single(zeros(480,752,3));
xyzPoints2=single(zeros(480,752,3));
xyzPoints3=single(zeros(480,752,3));

%get camera instrinsics and baseline values from camera info messages
focallength0 = Msgs8{1}.K(1);%camchain.cam0.intrinsics(1);
baseline0 = -T_cn_cnm1(1,4);
ccx0 = Msgs8{1}.K(3);%camchain.cam0.intrinsics(3);
ccy0 = Msgs8{1}.K(6);%camchain.cam0.intrinsics(4);

focallength1 = Msgs10{1}.K(1);
baseline1 = -T_cn_cnm3(1,4);
ccx1 = Msgs10{1}.K(3);
ccy1 = Msgs10{1}.K(6);

focallength2 = Msgs12{1}.K(1);
baseline2 = -T_cn_cnm5(1,4);
ccx2 = Msgs12{1}.K(3);
ccy2 = Msgs12{1}.K(6);

focallength3 = Msgs14{1}.K(1);
baseline3 = -T_cn_cnm7(1,4);
ccx3 = Msgs14{1}.K(3);
ccy3 = Msgs14{1}.K(6);

%crop last 3 bits of 8 bit disparity value (contains debug info)
for(y=1:1:480)
    for(x=1:1:752)
        image0_depth(y,x) =  bitand(image0_depth(y,x),248);%248 = 11111000bin
        image1_depth(y,x) =  bitand(image1_depth(y,x),248);%248 = 11111000bin
        image2_depth(y,x) =  bitand(image2_depth(y,x),248);%248 = 11111000bin
        image3_depth(y,x) =  bitand(image3_depth(y,x),248);%248 = 11111000bin      
    end
end
disparityMap0 = single((uint8(image0_depth))/8.0);
disparityMap1 = single((uint8(image1_depth))/8.0);
disparityMap2 = single((uint8(image2_depth))/8.0);
disparityMap3 = single((uint8(image3_depth))/8.0);

%adjust for extended disparity range with offset of 32 pixels and every 2nd
%sample only
disparityMap0 = disparityMap0*2+32;
disparityMap1 = disparityMap1*2+32;
disparityMap2 = disparityMap2*2+32;
disparityMap3 = disparityMap3*2+32;

%avoid division 0, crop last 4 disparity values
mindisp = 1*2+32;
%set constant to replace division 0
scaleddisp = 2*2+32*10000;

%compensate for x/y offset of disparity map
xshift = 22;
yshift = -8;

%crop image boarder regions
frame = 80;




for(y=1+frame:1:480-frame)
    for(x=1+frame:1:752-frame)
                if(disparityMap0(y,x,1) < mindisp)
                    xyzPoints0(y,x,3) = focallength0*baseline0/scaleddisp;
                else
                    xyzPoints0(y,x,3) = focallength0*baseline0/(disparityMap0(y,x,1));
                end
        xyzPoints0(y,x,1) = (x+xshift-ccx0)/focallength0*xyzPoints0(y,x,3);
        xyzPoints0(y,x,2) = (y+yshift-ccy0)/focallength0*xyzPoints0(y,x,3);
        
        %rotate/translate 3D points to cam0 rect reference frame
        temp = R_0*[xyzPoints0(y,x,1),xyzPoints0(y,x,2),xyzPoints0(y,x,3),1 ]';
        xyzPoints0(y,x,1) = temp(1);
        xyzPoints0(y,x,2) = temp(2);
        xyzPoints0(y,x,3) = temp(3);
    end
end

for(y=1+frame:1:480-frame)
    for(x=1+frame:1:752-frame)
               if(disparityMap1(y,x,1) <mindisp)
                    xyzPoints1(y,x,3) = focallength1*baseline1/scaleddisp;
                else
                    xyzPoints1(y,x,3) = focallength1*baseline1/(disparityMap1(y,x,1));
               end
        xyzPoints1(y,x,1) = (x+xshift-ccx1)/focallength1*xyzPoints1(y,x,3);
        xyzPoints1(y,x,2) = (y+yshift-ccy1)/focallength1*xyzPoints1(y,x,3);
        
        %rotate/translate 3D points to cam0 rect reference frame
        %temp = R_1*inv(T_cn_cnm2*T_cn_cnm1)*[xyzPoints1(y,x,1),xyzPoints1(y,x,2),xyzPoints1(y,x,3),1 ]';
        temp = R_1*(T_cn_cnm2*T_cn_cnm1)\[xyzPoints1(y,x,1),xyzPoints1(y,x,2),xyzPoints1(y,x,3),1 ]';
        xyzPoints1(y,x,1) = temp(1);
        xyzPoints1(y,x,2) = temp(2);
        xyzPoints1(y,x,3) = temp(3);
    end
end

% for(y=1+frame:1:480-frame)
%     for(x=1+frame:1:752-frame)
% 
%         xcompare = focallength1*xyzPoints0(y,x,1)/xyzPoints0(y,x,3)+ccx1;
%         ycompare = focallength1*xyzPoints0(y,x,2)/xyzPoints0(y,x,3)+ccy1; 
%         
%         if(xcompare<752 && xcompare>1 && ycompare<480 && ycompare>1)
%             
%             if(abs(xyzPoints0(y,x,3) - xyzPoints1(floor(ycompare),floor(xcompare),3))<0.02)
%                 %valid
%             else
%                 xyzPoints0(y,x,1) = 0;
%                 xyzPoints0(y,x,2) = 0;
%                 xyzPoints0(y,x,3) = 0;
%                 
%                 xyzPoints1(floor(ycompare),floor(xcompare),1) = 0;
%                 xyzPoints1(floor(ycompare),floor(xcompare),2) = 0;
%                 xyzPoints1(floor(ycompare),floor(xcompare),3) = 0;
%                 
%             end
%         end
%         
%     end
% end

for(y=1+frame:1:480-frame)
    for(x=1+frame:1:752-frame)
               if(disparityMap2(y,x,1) <mindisp)
                    xyzPoints2(y,x,3) = focallength2*baseline2/scaleddisp;
                else
                    xyzPoints2(y,x,3) = focallength2*baseline2/(disparityMap2(y,x,1));
               end
        xyzPoints2(y,x,1) = (x+xshift-ccx2)/focallength2*xyzPoints2(y,x,3);
        xyzPoints2(y,x,2) = (y+yshift-ccy2)/focallength2*xyzPoints2(y,x,3);
        
        %rotate/translate 3D points to cam0 rect reference frame
        temp = R_2*(T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1)\[xyzPoints2(y,x,1),xyzPoints2(y,x,2),xyzPoints2(y,x,3),1 ]';
        xyzPoints2(y,x,1) = temp(1);
        xyzPoints2(y,x,2) = temp(2);
        xyzPoints2(y,x,3) = temp(3);
    end
end

for(y=1+frame:1:480-frame)
    for(x=1+frame:1:752-frame)
               if(disparityMap3(y,x,1) < mindisp)
                    xyzPoints3(y,x,3) = focallength3*baseline3/scaleddisp;
                else
                    xyzPoints3(y,x,3) = focallength3*baseline3/(disparityMap3(y,x,1));
               end
        xyzPoints3(y,x,1) = (x+xshift-ccx3)/focallength3*xyzPoints3(y,x,3);
        xyzPoints3(y,x,2) = (y+yshift-ccy3)/focallength3*xyzPoints3(y,x,3);
        
        %rotate/translate 3D points to cam0 rect reference frame
        temp = R_3*(T_cn_cnm6*T_cn_cnm5*T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1)\[xyzPoints3(y,x,1),xyzPoints3(y,x,2),xyzPoints3(y,x,3),1 ]';
        xyzPoints3(y,x,1) = temp(1);
        xyzPoints3(y,x,2) = temp(2);
        xyzPoints3(y,x,3) = temp(3);
    end
end

%show all 4 pointclouds in 3dview
figure(2)
pcshow(xyzPoints0)
hold on;
pcshow(xyzPoints1)
pcshow(xyzPoints2)
pcshow(xyzPoints3)

%total translation/rotation of cameras w.r.t cam0
T0 = T_cn_cnm0;
T1 = inv(T_cn_cnm1*T_cn_cnm0);
T2 = inv(T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);0;
T3 = inv(T_cn_cnm3*T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);
T4 = inv(T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);
T5 = inv(T_cn_cnm5*T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);
T6 = inv(T_cn_cnm6*T_cn_cnm5*T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);
T7 = inv(T_cn_cnm7*T_cn_cnm6*T_cn_cnm5*T_cn_cnm4*T_cn_cnm3*T_cn_cnm2*T_cn_cnm1*T_cn_cnm0);

%plot camera positions in 3d view
size_cam = 0.012;
cam0 = plotCamera('Location',T0([1 2 3],4),'Orientation',T0([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','red');
cam1 = plotCamera('Location',T1([1 2 3],4),'Orientation',T1([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','blue');
cam2 = plotCamera('Location',T2([1 2 3],4),'Orientation',T2([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','green');
cam3 = plotCamera('Location',T3([1 2 3],4),'Orientation',T3([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','yellow');
cam4 = plotCamera('Location',T4([1 2 3],4),'Orientation',T4([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','black');
cam5 = plotCamera('Location',T5([1 2 3],4),'Orientation',T5([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','cyan');
cam6 = plotCamera('Location',T6([1 2 3],4),'Orientation',T6([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','magenta');
cam7 = plotCamera('Location',T7([1 2 3],4),'Orientation',T7([1 2 3],[1 2 3])','Opacity',0,'Size',size_cam,'Color','red');

%store combined pointcloud as .PLY file
xyzPointsTotal = [xyzPoints0,xyzPoints1,xyzPoints2,xyzPoints3];
ptCloud = pointCloud(xyzPointsTotal);
pcwrite(ptCloud,'ptCloudBox','PLYFormat','binary');


