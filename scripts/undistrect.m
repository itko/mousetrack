function [Rout] = undistrect(camchain)
% TODO: this is nearly the original sript from Dominik
% If anybody has time, feel free to clean it up by replacing the copy paste
% code with proper loops and so.


%T_cn_cnm0 = camchain.cam0.
T_cn_cnm1 = camchain.cam1.T_cn_cnm1;
T_cn_cnm2 = camchain.cam2.T_cn_cnm1;
T_cn_cnm3 = camchain.cam3.T_cn_cnm1;
T_cn_cnm4 = camchain.cam4.T_cn_cnm1;
T_cn_cnm5 = camchain.cam5.T_cn_cnm1;
T_cn_cnm6 = camchain.cam6.T_cn_cnm1;
T_cn_cnm7 = camchain.cam7.T_cn_cnm1;

nx = 752;
ny = 480;

zoom = 30;%adjust final focal length

fc_left = [camchain.cam0.intrinsics(1), camchain.cam0.intrinsics(2)];
fc_right =[camchain.cam1.intrinsics(1), camchain.cam1.intrinsics(2)];

cc_left = [camchain.cam0.intrinsics(3), camchain.cam0.intrinsics(4)];
cc_right =[camchain.cam1.intrinsics(3), camchain.cam1.intrinsics(4)];

kc_left = [camchain.cam0.distortion_coeffs, 0];
kc_right =[camchain.cam1.distortion_coeffs, 0];

alpha_c_left =  0.00000 ;
alpha_c_right = 0.00000 ;

G = (T_cn_cnm1);

T = [G(1,4) G(2,4) G(3,4)]';

R = [G(1,1) G(1,2) G(1,3);G(2,1) G(2,2) G(2,3);G(3,1) G(3,2) G(3,3)];

om = rodrigues(R);
%---------------------------------------------------------------------------------------

% Bring the 2 cameras in the same orientation by rotating them "minimally":
r_r = rodrigues(-om/2);
r_l = r_r';
t = r_r * T;

% Rotate both cameras so as to bring the translation vector in alignment with the (1;0;0) axis:
if abs(t(1)) > abs(t(2)),
    type_stereo = 0;
    uu = [1;0;0]; % Horizontal epipolar lines
else
    type_stereo = 1;
    uu = [0;1;0]; % Vertical epipolar lines
end;
if dot(uu,t)<0,
    uu = -uu; % Swtich side of the vector
end;
ww = cross(t,uu);
ww = ww/norm(ww);
ww = acos(abs(dot(t,uu))/(norm(t)*norm(uu)))*ww;
R2 = rodrigues(ww);


% Global rotations to be applied to both views:
R_R = R2 * r_r;
R_L = R2 * r_l;


% The resulting rigid motion between the two cameras after image rotations (substitutes of om, R and T):
R_new = eye(3);
om_new = zeros(3,1);
T_new = R_R*T;

% Computation of the *new* intrinsic parameters for both left and right cameras:

% Vertical focal length *MUST* be the same for both images (here, we are trying to find a focal length that retains as much information contained in the original distorted images):
if kc_left(1) < 0,
    fc_y_left_new = fc_left(2) * (1 + kc_left(1)*(nx^2 + ny^2)/(4*fc_left(2)^2));
else
    fc_y_left_new = fc_left(2);
end;
if kc_right(1) < 0,
    fc_y_right_new = fc_right(2) * (1 + kc_right(1)*(nx^2 + ny^2)/(4*fc_right(2)^2));
else
    fc_y_right_new = fc_right(2);
end;
fc_y_new = min(fc_y_left_new,fc_y_right_new)+zoom;


% For simplicity, let's pick the same value for the horizontal focal length as the vertical focal length (resulting into square pixels):
fc_left_new = round([fc_y_new;fc_y_new]);
fc_right_new = round([fc_y_new;fc_y_new]);
%fc_right_new = [fc_y_right_new,fc_y_right_new];
%fc_left_new = [fc_y_left_new,fc_y_left_new];

% Select the new principal points to maximize the visible area in the rectified images

cc_left_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_left,cc_left,kc_left,alpha_c_left);[1 1 1 1]],rodrigues(R_L),zeros(3,1),fc_left_new,[0;0],zeros(5,1),0),2);
cc_right_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_right,cc_right,kc_right,alpha_c_right);[1 1 1 1]],rodrigues(R_R),zeros(3,1),fc_right_new,[0;0],zeros(5,1),0),2);


% For simplivity, set the principal points for both cameras to be the average of the two principal points.
cc_y_new = (cc_left_new(2) + cc_right_new(2))/2;
cc_x_new = (cc_left_new(1) + cc_right_new(1))/2;

cc_left_new = [cc_x_new;cc_y_new];
cc_right_new = [cc_x_new;cc_y_new];

% Of course, we do not want any skew or distortion after rectification:
alpha_c_left_new = 0;
alpha_c_right_new = 0;
kc_left_new = zeros(5,1);
kc_right_new = zeros(5,1);

% The resulting left and right camera matrices:
KK_left_new = [fc_left_new(1) fc_left_new(1)*alpha_c_left_new cc_left_new(1);0 fc_left_new(2) cc_left_new(2); 0 0 1];
KK_right_new = [fc_right_new(1) fc_right_new(1)*alpha_c_right cc_right_new(1);0 fc_right_new(2) cc_right_new(2); 0 0 1];

% The sizes of the images are the same:
nx_right_new = nx;
ny_right_new = ny;
nx_left_new = nx;
ny_left_new = ny;

%---------------------------------------------------------------------------

KL = [fc_left(1) 0 cc_left(1);0 fc_left(2) cc_left(2);0 0 1];
KR = [fc_right(1) 0 cc_right(1);0 fc_right(2) cc_right(2);0 0 1];

Hleft =  KL*R_L'/(KK_left_new);%*inv(KK_left_new;
Hright = KR*R_R'/(KK_right_new);

R_0 = [[R_L [0;0;0]];0 ,0, 0, 1];
Rout{1} = R_0;

fc_left = [camchain.cam2.intrinsics(1), camchain.cam2.intrinsics(2)];
fc_right =[camchain.cam3.intrinsics(1), camchain.cam3.intrinsics(2)];

cc_left = [camchain.cam2.intrinsics(3), camchain.cam2.intrinsics(4)];
cc_right =[camchain.cam3.intrinsics(3), camchain.cam3.intrinsics(4)];

kc_left = [camchain.cam2.distortion_coeffs, 0];
kc_right =[camchain.cam3.distortion_coeffs, 0];

alpha_c_left =  0.00000 ;
alpha_c_right = 0.00000 ;

G = (T_cn_cnm3);

T = [G(1,4) G(2,4) G(3,4)]';

R = [G(1,1) G(1,2) G(1,3);G(2,1) G(2,2) G(2,3);G(3,1) G(3,2) G(3,3)];

om = rodrigues(R);
%---------------------------------------------------------------------------------------

% Bring the 2 cameras in the same orientation by rotating them "minimally":
r_r = rodrigues(-om/2);
r_l = r_r';
t = r_r * T;

% Rotate both cameras so as to bring the translation vector in alignment with the (1;0;0) axis:
if abs(t(1)) > abs(t(2)),
    type_stereo = 0;
    uu = [1;0;0]; % Horizontal epipolar lines
else
    type_stereo = 1;
    uu = [0;1;0]; % Vertical epipolar lines
end;
if dot(uu,t)<0,
    uu = -uu; % Swtich side of the vector
end;
ww = cross(t,uu);
ww = ww/norm(ww);
ww = acos(abs(dot(t,uu))/(norm(t)*norm(uu)))*ww;
R2 = rodrigues(ww);


% Global rotations to be applied to both views:
R_R = R2 * r_r;
R_L = R2 * r_l;


% The resulting rigid motion between the two cameras after image rotations (substitutes of om, R and T):
R_new = eye(3);
om_new = zeros(3,1);
T_new = R_R*T;

% Computation of the *new* intrinsic parameters for both left and right cameras:

% Vertical focal length *MUST* be the same for both images (here, we are trying to find a focal length that retains as much information contained in the original distorted images):
if kc_left(1) < 0,
    fc_y_left_new = fc_left(2) * (1 + kc_left(1)*(nx^2 + ny^2)/(4*fc_left(2)^2));
else
    fc_y_left_new = fc_left(2);
end;
if kc_right(1) < 0,
    fc_y_right_new = fc_right(2) * (1 + kc_right(1)*(nx^2 + ny^2)/(4*fc_right(2)^2));
else
    fc_y_right_new = fc_right(2);
end;
fc_y_new = min(fc_y_left_new,fc_y_right_new)+zoom;


% For simplicity, let's pick the same value for the horizontal focal length as the vertical focal length (resulting into square pixels):
fc_left_new = round([fc_y_new;fc_y_new]);
fc_right_new = round([fc_y_new;fc_y_new]);
%fc_right_new = [fc_y_right_new,fc_y_right_new];
%fc_left_new = [fc_y_left_new,fc_y_left_new];

% Select the new principal points to maximize the visible area in the rectified images

cc_left_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_left,cc_left,kc_left,alpha_c_left);[1 1 1 1]],rodrigues(R_L),zeros(3,1),fc_left_new,[0;0],zeros(5,1),0),2);
cc_right_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_right,cc_right,kc_right,alpha_c_right);[1 1 1 1]],rodrigues(R_R),zeros(3,1),fc_right_new,[0;0],zeros(5,1),0),2);


% For simplivity, set the principal points for both cameras to be the average of the two principal points.
cc_y_new = (cc_left_new(2) + cc_right_new(2))/2;
cc_x_new = (cc_left_new(1) + cc_right_new(1))/2;

cc_left_new = [cc_x_new;cc_y_new];
cc_right_new = [cc_x_new;cc_y_new];

% Of course, we do not want any skew or distortion after rectification:
alpha_c_left_new = 0;
alpha_c_right_new = 0;
kc_left_new = zeros(5,1);
kc_right_new = zeros(5,1);

% The resulting left and right camera matrices:
KK_left_new = [fc_left_new(1) fc_left_new(1)*alpha_c_left_new cc_left_new(1);0 fc_left_new(2) cc_left_new(2); 0 0 1];
KK_right_new = [fc_right_new(1) fc_right_new(1)*alpha_c_right cc_right_new(1);0 fc_right_new(2) cc_right_new(2); 0 0 1];

% The sizes of the images are the same:
nx_right_new = nx;
ny_right_new = ny;
nx_left_new = nx;
ny_left_new = ny;

%---------------------------------------------------------------------------

KL = [fc_left(1) 0 cc_left(1);0 fc_left(2) cc_left(2);0 0 1];
KR = [fc_right(1) 0 cc_right(1);0 fc_right(2) cc_right(2);0 0 1];

Hleft =  KL*R_L'/(KK_left_new);%*inv(KK_left_new;
Hright = KR*R_R'/(KK_right_new);

Rout{2} = [[R_L [0;0;0]];0 ,0, 0, 1];

fc_left = [camchain.cam4.intrinsics(1), camchain.cam4.intrinsics(2)];
fc_right =[camchain.cam5.intrinsics(1), camchain.cam5.intrinsics(2)];

cc_left = [camchain.cam4.intrinsics(3), camchain.cam4.intrinsics(4)];
cc_right =[camchain.cam5.intrinsics(3), camchain.cam5.intrinsics(4)];

kc_left = [camchain.cam4.distortion_coeffs, 0];
kc_right =[camchain.cam5.distortion_coeffs, 0];

alpha_c_left =  0.00000 ;
alpha_c_right = 0.00000 ;

G = (T_cn_cnm5);

T = [G(1,4) G(2,4) G(3,4)]';

R = [G(1,1) G(1,2) G(1,3);G(2,1) G(2,2) G(2,3);G(3,1) G(3,2) G(3,3)];

om = rodrigues(R);
%---------------------------------------------------------------------------------------

% Bring the 2 cameras in the same orientation by rotating them "minimally":
r_r = rodrigues(-om/2);
r_l = r_r';
t = r_r * T;

% Rotate both cameras so as to bring the translation vector in alignment with the (1;0;0) axis:
if abs(t(1)) > abs(t(2)),
    type_stereo = 0;
    uu = [1;0;0]; % Horizontal epipolar lines
else
    type_stereo = 1;
    uu = [0;1;0]; % Vertical epipolar lines
end;
if dot(uu,t)<0,
    uu = -uu; % Swtich side of the vector
end;
ww = cross(t,uu);
ww = ww/norm(ww);
ww = acos(abs(dot(t,uu))/(norm(t)*norm(uu)))*ww;
R2 = rodrigues(ww);


% Global rotations to be applied to both views:
R_R = R2 * r_r;
R_L = R2 * r_l;


% The resulting rigid motion between the two cameras after image rotations (substitutes of om, R and T):
R_new = eye(3);
om_new = zeros(3,1);
T_new = R_R*T;

% Computation of the *new* intrinsic parameters for both left and right cameras:

% Vertical focal length *MUST* be the same for both images (here, we are trying to find a focal length that retains as much information contained in the original distorted images):
if kc_left(1) < 0,
    fc_y_left_new = fc_left(2) * (1 + kc_left(1)*(nx^2 + ny^2)/(4*fc_left(2)^2));
else
    fc_y_left_new = fc_left(2);
end;
if kc_right(1) < 0,
    fc_y_right_new = fc_right(2) * (1 + kc_right(1)*(nx^2 + ny^2)/(4*fc_right(2)^2));
else
    fc_y_right_new = fc_right(2);
end;
fc_y_new = min(fc_y_left_new,fc_y_right_new)+zoom;


% For simplicity, let's pick the same value for the horizontal focal length as the vertical focal length (resulting into square pixels):
fc_left_new = round([fc_y_new;fc_y_new]);
fc_right_new = round([fc_y_new;fc_y_new]);
%fc_right_new = [fc_y_right_new,fc_y_right_new];
%fc_left_new = [fc_y_left_new,fc_y_left_new];

% Select the new principal points to maximize the visible area in the rectified images

cc_left_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_left,cc_left,kc_left,alpha_c_left);[1 1 1 1]],rodrigues(R_L),zeros(3,1),fc_left_new,[0;0],zeros(5,1),0),2);
cc_right_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_right,cc_right,kc_right,alpha_c_right);[1 1 1 1]],rodrigues(R_R),zeros(3,1),fc_right_new,[0;0],zeros(5,1),0),2);


% For simplivity, set the principal points for both cameras to be the average of the two principal points.
cc_y_new = (cc_left_new(2) + cc_right_new(2))/2;
cc_x_new = (cc_left_new(1) + cc_right_new(1))/2;

cc_left_new = [cc_x_new;cc_y_new];
cc_right_new = [cc_x_new;cc_y_new];

% Of course, we do not want any skew or distortion after rectification:
alpha_c_left_new = 0;
alpha_c_right_new = 0;
kc_left_new = zeros(5,1);
kc_right_new = zeros(5,1);

% The resulting left and right camera matrices:
KK_left_new = [fc_left_new(1) fc_left_new(1)*alpha_c_left_new cc_left_new(1);0 fc_left_new(2) cc_left_new(2); 0 0 1];
KK_right_new = [fc_right_new(1) fc_right_new(1)*alpha_c_right cc_right_new(1);0 fc_right_new(2) cc_right_new(2); 0 0 1];

% The sizes of the images are the same:
nx_right_new = nx;
ny_right_new = ny;
nx_left_new = nx;
ny_left_new = ny;

%---------------------------------------------------------------------------

KL = [fc_left(1) 0 cc_left(1);0 fc_left(2) cc_left(2);0 0 1];
KR = [fc_right(1) 0 cc_right(1);0 fc_right(2) cc_right(2);0 0 1];

Hleft =  KL*R_L'/(KK_left_new);%*inv(KK_left_new;
Hright = KR*R_R'/(KK_right_new);

Rout{3} = [[R_L [0;0;0]];0 ,0, 0, 1]

fc_left = [camchain.cam6.intrinsics(1), camchain.cam6.intrinsics(2)];
fc_right =[camchain.cam7.intrinsics(1), camchain.cam7.intrinsics(2)];

cc_left = [camchain.cam6.intrinsics(3), camchain.cam6.intrinsics(4)];
cc_right =[camchain.cam7.intrinsics(3), camchain.cam7.intrinsics(4)];

kc_left = [camchain.cam6.distortion_coeffs, 0];
kc_right =[camchain.cam7.distortion_coeffs, 0];

alpha_c_left =  0.00000 ;
alpha_c_right = 0.00000 ;

G = (T_cn_cnm7);

T = [G(1,4) G(2,4) G(3,4)]';

R = [G(1,1) G(1,2) G(1,3);G(2,1) G(2,2) G(2,3);G(3,1) G(3,2) G(3,3)];

om = rodrigues(R);
%---------------------------------------------------------------------------------------

% Bring the 2 cameras in the same orientation by rotating them "minimally":
r_r = rodrigues(-om/2);
r_l = r_r';
t = r_r * T;

% Rotate both cameras so as to bring the translation vector in alignment with the (1;0;0) axis:
if abs(t(1)) > abs(t(2)),
    type_stereo = 0;
    uu = [1;0;0]; % Horizontal epipolar lines
else
    type_stereo = 1;
    uu = [0;1;0]; % Vertical epipolar lines
end;
if dot(uu,t)<0,
    uu = -uu; % Swtich side of the vector
end;
ww = cross(t,uu);
ww = ww/norm(ww);
ww = acos(abs(dot(t,uu))/(norm(t)*norm(uu)))*ww;
R2 = rodrigues(ww);


% Global rotations to be applied to both views:
R_R = R2 * r_r;
R_L = R2 * r_l;


% The resulting rigid motion between the two cameras after image rotations (substitutes of om, R and T):
R_new = eye(3);
om_new = zeros(3,1);
T_new = R_R*T;

% Computation of the *new* intrinsic parameters for both left and right cameras:

% Vertical focal length *MUST* be the same for both images (here, we are trying to find a focal length that retains as much information contained in the original distorted images):
if kc_left(1) < 0,
    fc_y_left_new = fc_left(2) * (1 + kc_left(1)*(nx^2 + ny^2)/(4*fc_left(2)^2));
else
    fc_y_left_new = fc_left(2);
end;
if kc_right(1) < 0,
    fc_y_right_new = fc_right(2) * (1 + kc_right(1)*(nx^2 + ny^2)/(4*fc_right(2)^2));
else
    fc_y_right_new = fc_right(2);
end;
fc_y_new = min(fc_y_left_new,fc_y_right_new)+zoom;


% For simplicity, let's pick the same value for the horizontal focal length as the vertical focal length (resulting into square pixels):
fc_left_new = round([fc_y_new;fc_y_new]);
fc_right_new = round([fc_y_new;fc_y_new]);
%fc_right_new = [fc_y_right_new,fc_y_right_new];
%fc_left_new = [fc_y_left_new,fc_y_left_new];

% Select the new principal points to maximize the visible area in the rectified images

cc_left_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_left,cc_left,kc_left,alpha_c_left);[1 1 1 1]],rodrigues(R_L),zeros(3,1),fc_left_new,[0;0],zeros(5,1),0),2);
cc_right_new = [(nx-1)/2;(ny-1)/2] - mean(project_points2([normalize_pixel([0  nx-1 nx-1 0; 0 0 ny-1 ny-1],fc_right,cc_right,kc_right,alpha_c_right);[1 1 1 1]],rodrigues(R_R),zeros(3,1),fc_right_new,[0;0],zeros(5,1),0),2);


% For simplivity, set the principal points for both cameras to be the average of the two principal points.
cc_y_new = (cc_left_new(2) + cc_right_new(2))/2;
cc_x_new = (cc_left_new(1) + cc_right_new(1))/2;

cc_left_new = [cc_x_new;cc_y_new];
cc_right_new = [cc_x_new;cc_y_new];

% Of course, we do not want any skew or distortion after rectification:
alpha_c_left_new = 0;
alpha_c_right_new = 0;
kc_left_new = zeros(5,1);
kc_right_new = zeros(5,1);

% The resulting left and right camera matrices:
KK_left_new = [fc_left_new(1) fc_left_new(1)*alpha_c_left_new cc_left_new(1);0 fc_left_new(2) cc_left_new(2); 0 0 1];
KK_right_new = [fc_right_new(1) fc_right_new(1)*alpha_c_right cc_right_new(1);0 fc_right_new(2) cc_right_new(2); 0 0 1];

% The sizes of the images are the same:
nx_right_new = nx;
ny_right_new = ny;
nx_left_new = nx;
ny_left_new = ny;

%---------------------------------------------------------------------------

KL = [fc_left(1) 0 cc_left(1);0 fc_left(2) cc_left(2);0 0 1];
KR = [fc_right(1) 0 cc_right(1);0 fc_right(2) cc_right(2);0 0 1];

Hleft =  KL*R_L'/(KK_left_new);%*inv(KK_left_new;
Hright = KR*R_R'/(KK_right_new);

Rout{4} = [[R_L [0;0;0]];0 ,0, 0, 1];


end
