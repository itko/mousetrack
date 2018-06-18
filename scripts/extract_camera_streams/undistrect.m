function [Rout] = undistrect(camchain)

T_cn_cnm{1} = eye(4);
T_cn_cnm{2} = camchain.cam1.T_cn_cnm1;
T_cn_cnm{3} = camchain.cam2.T_cn_cnm1;
T_cn_cnm{4} = camchain.cam3.T_cn_cnm1;
T_cn_cnm{5} = camchain.cam4.T_cn_cnm1;
T_cn_cnm{6} = camchain.cam5.T_cn_cnm1;
T_cn_cnm{7} = camchain.cam6.T_cn_cnm1;
T_cn_cnm{8} = camchain.cam7.T_cn_cnm1;

Rout{1} = undistrectPart(T_cn_cnm{2});

Rout{2} = undistrectPart(T_cn_cnm{4});

Rout{3} = undistrectPart(T_cn_cnm{6});

Rout{4} = undistrectPart(T_cn_cnm{8});
    
end

function [R] = undistrectPart(G)

T = G(1:3, 4);

R = G(1:3, 1:3);

om = rotationMatrixToVector(R);
%---------------------------------------------------------------------------------------

% Bring the 2 cameras in the same orientation by rotating them "minimally":
r_r = rotationVectorToMatrix(-om/2);
r_l = r_r';
t = r_r * T;

% Rotate both cameras so as to bring the translation vector in alignment with the (1;0;0) axis:
if abs(t(1)) > abs(t(2))
    uu = [1;0;0]; % Horizontal epipolar lines
else
    uu = [0;1;0]; % Vertical epipolar lines
end

if dot(uu,t)<0
    uu = -uu; % Swtich side of the vector
end
ww = cross(t,uu);
ww = ww/norm(ww);
ww = acos(abs(dot(t,uu))/(norm(t)*norm(uu)))*ww;

R2 = rotationVectorToMatrix(-ww);

% Global rotations to be applied to both views:
R_L = R2 * r_l;

R = [[R_L [0;0;0]];0 ,0, 0, 1];

end