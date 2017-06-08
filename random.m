function [ Err ] = random( N_LINES, NOISE_SIGMA )
%RANDOM Summary of this function goes here
%   Detailed explanation goes here
%%%
% Test of all methods using randomly generated lines.
%%%
%% TEST PARAMETERS
CUBE_SIZE = 10.0; % size of one side of an imaginary cube inside of which the lines are generated
CAM_DIST  = 1.0; % distance of the camera from the center of the imaginary cube
IMG_SIZE = [480 640];
FOCAL = 800;

%% GROUND-TRUTH camera pose (randomly generated)

% camera position
cam_X_GT = 10*rand()*pi/180;
cam_Y_GT = rand()*pi/180;
cam_Z_GT = rand()*pi/180;

norm_coef = CAM_DIST / norm([cam_X_GT cam_Y_GT cam_Z_GT]);

cam_X_GT = norm_coef * cam_X_GT;
cam_Y_GT = norm_coef * cam_Y_GT;
cam_Z_GT = norm_coef * cam_Z_GT;

T_GT = getTranslationVector(cam_X_GT, cam_Y_GT, cam_Z_GT);

% camera orientation
% if(rand < 0.5)
	cam_Gamma_GT = 2*rand()*pi/180;
	cam_Beta_GT  = 2*rand()*pi/180;
	cam_Alpha_GT = 2*rand()*pi/180;
% else
% 	cam_Gamma_GT = atan2(cam_Y_GT, cam_X_GT);
% 	cam_Beta_GT  = acos((cam_Z_GT - 0) / CAM_DIST);
% 	cam_Alpha_GT = 0;
% end

R_GT = getRotationMatrix(cam_Alpha_GT, cam_Beta_GT, cam_Gamma_GT);

% camera intrinsics
principal_x = (IMG_SIZE(2)-1) / 2;
principal_y = (IMG_SIZE(1)-1) / 2;
C  = getCameraMatrix(FOCAL, principal_x, principal_y);

TM_GT = R_GT * [eye(3) T_GT];


%% 3D line segment endpoints
% randomly generated
X_W = [CUBE_SIZE * rand(3, 2*N_LINES) - (CUBE_SIZE/2); ones(1, 2*N_LINES)];

%% 2D line segment endpoints

% Project 3D endpoints onto the normalized image plane using generated
% ground-truth camera pose
x_c = TM_GT * X_W;
for i = 1:3
	x_c(i,:) = x_c(i,:) ./ x_c(3,:);
end

% Convert 2D endpoint coordinates from normalized coords. to pixel coords.
x_img =  C * x_c;
for i = 1:3
	x_img(i,:) = x_img(i,:) ./ x_img(3,:);
end

% Add gaussian noise to the image endpoints
x_img_noisy = x_img;
x_img_noisy(1:2, :) = x_img(1:2, :) + (NOISE_SIGMA * randn(2, 2*N_LINES));

% Convert 2D endpoint coordinates back from pixel coords. to normalized coords.
x_c_noisy = inv(C) * x_img_noisy;

%% Camera pose estimation
% disp('Ground-truth pose [R|T]');
% disp([R_GT T_GT]);
% fprintf('\n');

if ((N_LINES >= 4) && (N_LINES <= 50))
% 	disp('Ansar [R|T]');
	[R_Ansar, T_Ansar] = Ansar_wrapper(X_W, x_c_noisy);
	[ER_Ansar, ET_Ansar, Ep_Ansar] = errors(R_Ansar, T_Ansar, R_GT, T_GT, X_W);
    err1 = [180*ER_Ansar/pi,  ET_Ansar, Ep_Ansar];
% 	disp([R_Ansar T_Ansar]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Ansar/pi,  ET_Ansar, Ep_Ansar);
else 
    err1 = [NaN NaN NaN];
end

if (N_LINES >= 3)
% 	disp('Mirzaei [R|T]');
	[R_Mirzaei, T_Mirzaei] = Mirzaei_wrapper(X_W, x_c_noisy);
	[ER_Mirzaei, ET_Mirzaei, Ep_Mirzaei] = errors(R_Mirzaei, T_Mirzaei, R_GT, T_GT, X_W);
    err2 = [180*ER_Mirzaei/pi,  ET_Mirzaei, Ep_Mirzaei];
% 	disp([R_Mirzaei T_Mirzaei]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Mirzaei/pi,  ET_Mirzaei, Ep_Mirzaei);
else 
    err2 = [NaN NaN NaN];
end

if (N_LINES >= 4)
% 	disp('RPnL [R|T]');
% 	[R_RPnL, T_RPnL] = RPnL_wrapper(X_W, x_c_noisy);
% 	[ER_RPnL, ET_RPnL, Ep_RPnL] = errors(R_RPnL, T_RPnL, R_GT, T_GT, X_W);
%     err3 = [180*ER_RPnL/pi,  ET_RPnL, Ep_RPnL];
    err3 = [0, 0, 0];
else 
    err3 = [NaN NaN NaN];
% 	disp([R_RPnL T_RPnL]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_RPnL/pi,  ET_RPnL, Ep_RPnL);
end

if (N_LINES >= 3)
% 	disp('ASPnL [R|T]');
	[R_ASPnL, T_ASPnL] = ASPnL_wrapper(X_W, x_c_noisy);
	[ER_ASPnL, ET_ASPnL, Ep_ASPnL] = errors(R_ASPnL, T_ASPnL, R_GT, T_GT, X_W);
    err4 = [180*ER_ASPnL/pi,  ET_ASPnL, Ep_ASPnL];
% 	disp([R_ASPnL T_ASPnL]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ASPnL/pi,  ET_ASPnL, Ep_ASPnL);
else 
    err4 = [NaN NaN NaN];
end

if (N_LINES >= 6)
% 	disp('LPnL_Bar_LS [R|T]');
	[R_LS, T_LS] = LPnL_Bar_LS_wrapper(X_W, x_c_noisy);
	[ER_LS, ET_LS, Ep_LS] = errors(R_LS, T_LS, R_GT, T_GT, X_W);
    err5 = [180*ER_LS/pi,  ET_LS, Ep_LS];
% 	disp([R_LS T_LS]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_LS/pi,  ET_LS, Ep_LS);
else 
    err5 = [NaN NaN NaN];  
end

if (N_LINES >= 4)
% 	disp('LPnL_Bar_ENull [R|T]');
	[R_ENull, T_ENull] = LPnL_Bar_ENull_wrapper(X_W, x_c_noisy);
	[ER_ENull, ET_ENull, Ep_ENull] = errors(R_ENull, T_ENull, R_GT, T_GT, X_W);
    err6 = [180*ER_ENull/pi,  ET_ENull, Ep_ENull];
% 	disp([R_ENull T_ENull]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ENull/pi,  ET_ENull, Ep_ENull);
else 
    err6 = [NaN NaN NaN];
end

if (N_LINES >= 6)
% 	disp('DLT-Lines [R|T]');
	[R_Lines, T_Lines] = DLT_Lines(X_W, x_c_noisy);
	[ER_Lines, ET_Lines, Ep_Lines] = errors(R_Lines, T_Lines, R_GT, T_GT, X_W);
    err7 = [ 180*ER_Lines/pi,  ET_Lines, Ep_Lines];
% 	disp([R_Lines T_Lines]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Lines/pi,  ET_Lines, Ep_Lines);
else 
    err7 = [NaN NaN NaN];
end

if (N_LINES >= 9)
% 	disp('DLT-Plücker-Lines [R|T]');
	[R_Plucker, T_Plucker] = DLT_Plucker_Lines(X_W, x_c_noisy);
	[ER_Plucker, ET_Plucker, Ep_Plucker] = errors(R_Plucker, T_Plucker, R_GT, T_GT, X_W);
    err8 = [180*ER_Plucker/pi,  ET_Plucker, Ep_Plucker];
% 	disp([R_Plucker T_Plucker]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Plucker/pi,  ET_Plucker, Ep_Plucker);
else 
    err8 = [NaN NaN NaN];
end

if (N_LINES >= 5)
% 	disp('DLT-Combined-Lines [R|T]');
	[R_Combi, T_Combi] = DLT_Combined_Lines(X_W, x_c_noisy);
	[ER_Combi, ET_Combi, Ep_Combi] = errors(R_Combi, T_Combi, R_GT, T_GT, X_W);
    err9 = [ 180*ER_Combi/pi,  ET_Combi, Ep_Combi];
% 	disp([R_Combi T_Combi]);
% 	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% 	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);
else 
    err9 = [NaN NaN NaN];
end

cam = eye(3);
%     [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c_noisy,cam, cam, zeros(3,1));
    [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c_noisy, cam, R_Combi, R_Combi*T_Combi);
[ER_Combi, ET_Combi, Ep_Combi] = errors(R_BA, R_BA'* T_BA, R_GT, T_GT, X_W);
err10 = [ 180*ER_Combi/pi,  ET_Combi, Ep_Combi];

if (norm(err10)>1)
    Err = 0;
else
    Err = [err1;err2;err3;err4;err5;err6;err7;err8;err9;err10;];
end
% fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
% fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);

end

