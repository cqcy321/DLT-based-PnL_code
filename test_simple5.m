%%%
% A minimal test of the DLT-Combined-Lines method with 5 predefined lines.
%%%

startup;

% ground-truth camera position: x:5.4114 y:5.0692 z:-23.875
cam_X_GT =   1 * rand;
cam_Y_GT =   1 * rand;
cam_Z_GT =   1 * rand;
T_GT = getTranslationVector(cam_X_GT, cam_Y_GT, cam_Z_GT);

% ground-truth camera orientation: alfa:-2.8405 beta:0 gamma:-0.81804 [rad]
cam_Alpha_GT =  0.1 * rand;
cam_Beta_GT  =  0.1 * rand;
cam_Gamma_GT =  0.1 * rand;
R_GT = getRotationMatrix(cam_Alpha_GT, cam_Beta_GT, cam_Gamma_GT);

%% Input

% 3D line segment endpoints (format: start point1; end point1; start point2...)
X_W = [ ...
	-0.20515		-2.6821			-1.0371 		1; ...
	 2.0508			 0.58559		 2.5663 		1; ...
	 4.9548			 4.6243			 0.35067		1; ...
	 4.6387			-3.8437			-4.4855 		1; ...
	-1.9565			 0.80192		 0.30964		1; ...
	 4.0121			 0.4055			-0.68019		1; ...
	 0.42667		 2.1241			-4.8333 		1; ...
	 3.0092			-3.5749			-0.21526		1; ...
	-2.4316			-1.3091			 1.6176 		1; ...
	-3.3039			-2.2122			-3.0178 		1 ...
]';

% 2D line segment endpoints (format: start point1; end point1; start point2...)
T_motion = [R_GT R_GT * T_GT];
c = T_motion * X_W;
x_c = [c(1,:)./c(3,:) ;c(2,:)./c(3,:);c(3,:)./c(3,:)];

cam = [700 0 320;0 700 240;0 0 1];
x_i = proj(cam * x_c)+normrnd(0, 1, 2, size(x_c,2));
x_c = cam \ [x_i; ones(1,size(x_i,2))];

% x_c = x_c + [normrnd(0, 0.01, 2, size(x_c,2)); zeros(1, size(x_c,2)) ];
% x_c = [...
% 	-0.073874			-0.089508		1; ...
% 	-0.036249			 0.095685		1; ...
% 	-0.00053754		 0.28196		1; ...
% 	-0.29165			-0.029618		1; ...
% 	 0.075238			-0.029277		1; ...
% 	-0.10457			 0.12219		1; ...
% 	 0.063366			 0.012622		1; ...
% 	-0.18764			-0.012086		1; ...
% 	 0.025864			-0.075711		1; ...
% 	 0.027674			-0.19939		1 ...
% ]';

%% Camera pose estimation
disp('Ground-truth pose [R|T]');
disp([R_GT T_GT]);

disp('DLT-Combined-Lines [R|T]');
[R_Combi, T_Combi] = DLT_Combined_Lines(X_W, x_c);
[ER_Combi, ET_Combi, Ep_Combi] = errors(R_Combi, T_Combi, R_GT, T_GT, X_W);
disp([R_Combi T_Combi]);
fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);

cam = eye(3);
    [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c,cam, cam, zeros(3,1));
    [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c,cam, R_Combi, T_Combi);
disp('Eric [R|T]');
[R_BA  R_BA'*T_BA]
[ER_Combi, ET_Combi, Ep_Combi] = errors(R_BA, R_BA'* T_BA, R_GT, T_GT, X_W);

fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);
return;
