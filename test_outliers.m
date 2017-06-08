%%%
% Test all methods using randomly generated lines with outliers.
%%%

startup;

%% TEST PARAMETERS
N_LINES = 500; % how many lines to generate
N_OUTLIERS = 0.1*N_LINES;
NOISE_SIGMA = 2.0;
OUTL_SIGMA = 100.0;

if(NOISE_SIGMA >= 1.0)
	RANSAC_REPROJ_ERR_TH = 2e-6 * (log(NOISE_SIGMA) / log(1.5) + 0.5);
else
	RANSAC_REPROJ_ERR_TH = 2e-17;
end
AOR_REPROJ_ERR_TH = NOISE_SIGMA;
RANSAC_P = 0.99;
RANSAC_MAX_ITER = 1e4;

CUBE_SIZE = 10;
CAM_DIST = 25.0;
IMG_SIZE = [480 640];
FOCAL = 800;

%% Check parameter values
if (~(N_LINES >= N_OUTLIERS))
	error('The number of outliers N_OUTLIERS cannot exceed the number of all lines N_LINES.')
end

%% GROUND-TRUTH camera pose (randomly generated)
% camera position
cam_X_GT = 2*rand()-1;
cam_Y_GT = 2*rand()-1;
cam_Z_GT = 2*rand()-1;

norm_coef = CAM_DIST / norm([cam_X_GT cam_Y_GT cam_Z_GT]);

cam_X_GT = norm_coef * cam_X_GT;
cam_Y_GT = norm_coef * cam_Y_GT;
cam_Z_GT = norm_coef * cam_Z_GT;

T_GT = getTranslationVector(cam_X_GT, cam_Y_GT, cam_Z_GT);

% camera orientation
if(rand < 0.5)
	cam_Gamma_GT = -atan2(cam_X_GT, cam_Y_GT);
	cam_Beta_GT = 0;
	cam_Alpha_GT = -acos((cam_Z_GT - 0) / CAM_DIST);
else
	cam_Gamma_GT = atan2(cam_Y_GT, cam_X_GT);
	cam_Beta_GT = acos((cam_Z_GT - 0) / CAM_DIST);
	cam_Alpha_GT = 0;
end

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
	x_c(i,:)  = x_c(i,:)  ./ x_c(3,:);
end

% Convert 2D endpoint coordinates from normalized coords. to pixel coords.
x_img = C * x_c;
for i = 1:3
	x_img(i,:) = x_img(i,:) ./ x_img(3,:);
end

% Add gaussian noise to the image endpoints
x_img_noisy = x_img;
x_img_noisy(1:2, :) = x_img_noisy(1:2, :) + (NOISE_SIGMA * randn(2, 2*N_LINES));

% Produce outliers by additional strong perturbation of image line endpoints
x_img_noisy(1:2, end-2*N_OUTLIERS+1:end) = x_img_noisy(1:2, end-2*N_OUTLIERS+1:end) + (OUTL_SIGMA * randn(2, 2*N_OUTLIERS));

x_c_noisy = inv(C) * x_img_noisy;

%% Camera pose estimation with outlier rejection
disp('Ground-truth pose [R|T]');
disp([R_GT T_GT]);
fprintf('\n');

if (N_LINES >= 4)
	disp('Ansar + MLESAC4 + RPnL [R|T]');
	[R_Ansar, T_Ansar] = Ansar_MLESAC4_RPnL(X_W, x_c_noisy, RANSAC_P, RANSAC_MAX_ITER, RANSAC_REPROJ_ERR_TH);
	[ER_Ansar, ET_Ansar, Ep_Ansar] = errors(R_Ansar, T_Ansar, R_GT, T_GT, X_W);
	disp([R_Ansar T_Ansar]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Ansar/pi,  ET_Ansar, Ep_Ansar);
end

if (N_LINES >= 3)
	disp('Mirzaei + MLESAC3 [R|T]');
	[R_Mirzaei, T_Mirzaei] = Mirzaei_MLESAC3(X_W, x_c_noisy, RANSAC_P, RANSAC_MAX_ITER, RANSAC_REPROJ_ERR_TH);
	[ER_Mirzaei, ET_Mirzaei, Ep_Mirzaei] = errors(R_Mirzaei, T_Mirzaei, R_GT, T_GT, X_W);
	disp([R_Mirzaei T_Mirzaei]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Mirzaei/pi,  ET_Mirzaei, Ep_Mirzaei);
end

if (N_LINES >= 4)
	disp('RPnL + MLESAC4 [R|T]');
	[R_RPnL, T_RPnL] = RPnL_MLESAC4(X_W, x_c_noisy, RANSAC_P, RANSAC_MAX_ITER, RANSAC_REPROJ_ERR_TH);
	[ER_RPnL, ET_RPnL, Ep_RPnL] = errors(R_RPnL, T_RPnL, R_GT, T_GT, X_W);
	disp([R_RPnL T_RPnL]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_RPnL/pi,  ET_RPnL, Ep_RPnL);
end

if (N_LINES >= 3)
	disp('P3L + RANSAC3 [R|T]');
	[R_P3L, T_P3L] = P3L_RANSAC3_wrapper(X_W, x_c_noisy);
	[ER_P3L, ET_P3L, Ep_P3L] = errors(R_P3L, T_P3L, R_GT, T_GT, X_W);
	disp([R_P3L T_P3L]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_P3L/pi,  ET_P3L, Ep_P3L);
end

if (N_LINES >= 4)
	disp('ASPnL + RANSAC4 [R|T]');
	[R_ASPnL, T_ASPnL] = ASPnL_RANSAC4_wrapper(X_W, x_c_noisy);
	[ER_ASPnL, ET_ASPnL, Ep_ASPnL] = errors(R_ASPnL, T_ASPnL, R_GT, T_GT, X_W);
	disp([R_ASPnL T_ASPnL]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ASPnL/pi,  ET_ASPnL, Ep_ASPnL);
end

if (N_LINES >= 6)
	disp('LPnL_Bar_LS + AOR [R|T]');
	[R_LS, T_LS] = LPnL_Bar_LS_AOR_wrapper(X_W, x_c_noisy);
	[ER_LS, ET_LS, Ep_LS] = errors(R_LS, T_LS, R_GT, T_GT, X_W);
	disp([R_LS T_LS]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_LS/pi,  ET_LS, Ep_LS);
end

if (N_LINES >= 4)
	disp('LPnL_Bar_ENull + AOR [R|T]');
	[R_ENull, T_ENull] = LPnL_Bar_ENull_AOR_wrapper(X_W, x_c_noisy);
	[ER_ENull, ET_ENull, Ep_ENull] = errors(R_ENull, T_ENull, R_GT, T_GT, X_W);
	disp([R_ENull T_ENull]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ENull/pi,  ET_ENull, Ep_ENull);
end

if (N_LINES >= 6)
	disp('DLT-Lines + AOR [R|T]');
	[R_Lines, T_Lines] = DLT_Lines_AOR(X_W, x_c_noisy, AOR_REPROJ_ERR_TH, FOCAL);
	[ER_Lines, ET_Lines, Ep_Lines] = errors(R_Lines, T_Lines, R_GT, T_GT, X_W);
	disp([R_Lines T_Lines]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Lines/pi,  ET_Lines, Ep_Lines);
end

if (N_LINES >= 9)
	disp('DLT-Plücker-Lines + AOR [R|T]');
	[R_Plucker, T_Plucker] = DLT_Plucker_Lines_AOR(X_W, x_c_noisy, AOR_REPROJ_ERR_TH, FOCAL);
	[ER_Plucker, ET_Plucker, Ep_Plucker] = errors(R_Plucker, T_Plucker, R_GT, T_GT, X_W);
	disp([R_Plucker T_Plucker]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Plucker/pi,  ET_Plucker, Ep_Plucker);
end

if (N_LINES >= 5)
	disp('DLT-Combined-Lines + AOR [R|T]');
	[R_Combi, T_Combi] = DLT_Combined_Lines_AOR(X_W, x_c_noisy, AOR_REPROJ_ERR_TH, FOCAL);
	[ER_Combi, ET_Combi, Ep_Combi] = errors(R_Combi, T_Combi, R_GT, T_GT, X_W);
	disp([R_Combi T_Combi]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[]\n');
	fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);
end

