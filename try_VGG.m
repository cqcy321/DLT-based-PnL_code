%%%
% Test of the methods on the VGG multiview dataset.
%%%

%% Settings for individual datasets - please uncomment/comment as needed.

startup;

% Comment this and uncomment the desired dataset
% warning('Please comment this and uncomment a dataset of your choice in this script.'); return;

% % Model House
% folder_path = ['datasets' filesep 'VGG' filesep 'Model-House' filesep];
% L_file = 'house.l3d';
% line_matches_file = 'house.nview-lines';
% img1_nr = 0;
% N_IMGS = 10;
% img_part_name = 'house.';
% img_ext = '.pgm';
% T_FORM_3D = diag([-1 -1 -1  1]);
% T_FORM_2D = diag([1 1 1]);

% Corridor
folder_path = ['datasets' filesep 'VGG' filesep 'Corridor' filesep];
L_file = 'bt.l3d';
line_matches_file = 'bt_original.nview-lines';
img1_nr = 0;
N_IMGS = 11;
img_part_name = 'bt.';
img_ext = '.pgm';
T_FORM_3D = diag([-1 1 -1  1]);
% T_FORM_3D = eye(4);
T_FORM_2D = diag([1 -1 1]);

% % Merton College I
% folder_path = ['datasets' filesep 'VGG' filesep 'Merton-College-I' filesep];
% L_file = 'l3d';
% line_matches_file = 'nview-lines';
% img1_nr = 1;
% N_IMGS = 3;
% img_part_name = '';
% img_ext = '.jpg';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([-1  1  1]);

% % Merton College II
% folder_path = ['datasets' filesep 'VGG' filesep 'Merton-College-II' filesep];
% L_file = 'l3d';
% line_matches_file = 'nview-lines';
% img1_nr = 1;
% N_IMGS = 3;
% img_part_name = '';
% img_ext = '.jpg';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([-1  1  1]);

% % Merton College III
% folder_path = ['datasets' filesep 'VGG' filesep 'Merton-College-III' filesep];
% L_file = 'l3d';
% line_matches_file = 'nview-lines';
% img1_nr = 1;
% N_IMGS = 3;
% img_part_name = '';
% img_ext = '.jpg';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([-1  1  1]);

% % University Library
% folder_path = ['datasets' filesep 'VGG' filesep 'University-Library' filesep];
% L_file = 'l3d';
% line_matches_file = 'nview-lines';
% img1_nr = 1;
% N_IMGS = 3;
% img_part_name = '';
% img_ext = '.jpg';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([-1  1  1]);

% % Wadham College
% folder_path = ['datasets' filesep 'VGG' filesep 'Wadham-College' filesep];
% L_file = 'l3d';
% line_matches_file = 'nview-lines';
% img1_nr = 1;
% N_IMGS = 5;
% img_part_name = '';
% img_ext = '.jpg';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([1 1 1]);

%% Run camera pose estimation on all images from a dataset
img_nrs = img1_nr : (N_IMGS + img1_nr - 1);

for img_nr = img_nrs
	
	l_file   = [img_part_name sprintf('%03d', img_nr) '.lines'];
	P_file   = [img_part_name sprintf('%03d', img_nr) '.P'];
	img_name = [img_part_name sprintf('%03d', img_nr) img_ext];
	
	fprintf('=== Processing image %s =======================\n\n', img_name);
	
	%% Read 3D line endpoints
	X_W = dlmread([folder_path L_file]);
	X_W = X_W';
	
	%% Read 3D-2D line matches
	f = fopen([folder_path line_matches_file], 'r');
	matches = textscan(f, '%s', 'EndOfLine', '\n');
	fclose(f);
	matches = matches{1,1};

	for i=1:length(matches(:))
		num = str2num(matches{i});
		if(~isempty(num))
			matches{i} = num;
		else
			matches{i} = NaN;
		end
	end

	matches = cell2mat(matches);
	matches = reshape(matches, N_IMGS, size(X_W,2));
	matches = matches';

	% keep just matches for current image
	matches = matches(:, (img_nr - img1_nr + 1));
	
	N_LINES = sum(~isnan(matches));
	
	%% Filter our nonmatched 3D lines
	X_W = X_W(:, ~isnan(matches));

	X_W_tmp = X_W;
	X_W = ones(4, 2*N_LINES);
	X_W(1:3, 1:2:end) = X_W_tmp(1:3, :);
	X_W(1:3, 2:2:end) = X_W_tmp(4:6, :);
	
	%% Read camera ground-truth parameters
	P_GT = dlmread([folder_path P_file]);
	
	% dataset-specific transformation of 3D coordinates
	X_W = T_FORM_3D * X_W;
	P_GT = P_GT * inv(T_FORM_3D);
	
	% decompose the projection matrix
	[C_GT, R_GT_VGG, T_GT_VGG] = vgg_KR_from_P(P_GT);
	
	% dataset-specific transformation of 2D coordinates
	C_GT = C_GT * T_FORM_2D;
	R_GT_VGG = inv(T_FORM_2D) * R_GT_VGG;
	if (det(R_GT_VGG) < 0)
		R_GT_VGG = -R_GT_VGG; % det(R_GT_VGG) must be +1!
	end
	
	R_GT =  R_GT_VGG;
	T_GT = -T_GT_VGG;
%     T_GT = diag([-1,1,-1]) *T_GT;
	
	%% Read 2D lines
	x_img = dlmread([folder_path l_file]);
	x_img = x_img(:, 1:4);
	x_img = x_img';

	% filter out nonmatched 2D lines
	x_img = x_img(:, matches(~isnan(matches)) + 1);

	x_img_tmp = x_img;
	x_img = ones(3, 2*N_LINES);
	x_img(1:2, 1:2:end) = x_img_tmp(1:2,:);
	x_img(1:2, 2:2:end) = x_img_tmp(3:4,:);

	x_c = inv(C_GT) * x_img;
	
	%% Camera pose estimation
	
	disp('Ground truth pose [R|T]');
	disp([R_GT T_GT]);
	fprintf('\n');
%     
    if (N_LINES >= 4)
		disp('RPnL [R|T]');
		[R_RPnL, T_RPnL] = RPnL_wrapper(X_W,x_c);
        
		[ER_RPnL, ET_RPnL, Ep_RPnL] = errors(R_RPnL, R_RPnL*T_RPnL, R_GT, T_GT, X_W);
		disp([R_RPnL R_RPnL*T_RPnL]);
		fprintf('\tOrient.err[°]   Pos.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_RPnL/pi,  ET_RPnL,Ep_RPnL);
    end
    
%     display_3D_lines([R_GT T_GT]*X_W,[R_RPnL R_RPnL*T_RPnL]*X_W);
%     errrrorrrr = proj(x_c) - proj([R_RPnL R_RPnL*T_RPnL] * X_W)
    
    R_estim = diag([-1  1  -1]);
    R_estim = eye(3);
    cam = eye(3);
    t_estim = -rand(3,1);
%     [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c,cam, R_estim,  t_estim);
    
    [R_BA, T_BA]= linePoseEstim_3e(X_W, x_c,cam, R_RPnL, R_RPnL*T_RPnL);
    disp('Yucao [R|T]');
%     T_BA = R_BA \ T_BA;
display_3D_lines([R_GT T_GT]*X_W,[R_BA T_BA]*X_W);
    [R_BA T_BA]
    [ER_Ansar, ET_Ansar, Ep_Ansar] = errors(R_BA, T_BA, R_GT, T_GT, X_W);
    		fprintf('\tOrient.err[°]   Pos.err[] \n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Ansar/pi,  ET_Ansar,Ep_RPnL);
	
% 	if ((N_LINES >= 4) && (N_LINES <= 50))
% 		disp('Ansar [R|T]');
% 		[R_Ansar, T_Ansar] = Ansar_wrapper(X_W, x_c);
% 		[ER_Ansar, ET_Ansar, Ep_Ansar] = errors(R_Ansar, T_Ansar, R_GT, T_GT, X_W);
% 		disp([R_Ansar T_Ansar]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Ansar/pi,  ET_Ansar, Ep_Ansar);
% 	end
% 
% 	if (N_LINES >= 3)
% 		disp('Mirzaei [R|T]');
% 		[R_Mirzaei, T_Mirzaei] = Mirzaei_wrapper(X_W, x_c);
% 		[ER_Mirzaei, ET_Mirzaei, Ep_Mirzaei] = errors(R_Mirzaei, T_Mirzaei, R_GT, T_GT, X_W);
% 		disp([R_Mirzaei T_Mirzaei]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Mirzaei/pi,  ET_Mirzaei, Ep_Mirzaei);
% 	end
% 
% 	if (N_LINES >= 4)
% 		disp('RPnL [R|T]');
% 		[R_RPnL, T_RPnL] = RPnL_wrapper(X_W, x_c);
% 		[ER_RPnL, ET_RPnL, Ep_RPnL] = errors(R_RPnL, T_RPnL, R_GT, T_GT, X_W);
% 		disp([R_RPnL T_RPnL]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_RPnL/pi,  ET_RPnL, Ep_RPnL);
% 	end
% 
% 	if (N_LINES >= 3)
% 		disp('ASPnL [R|T]');
% 		if (strcmp(folder_path, ['datasets' filesep 'VGG' filesep 'Corridor' filesep]))
% 			[R_ASPnL, T_ASPnL] = ASPnL_wrapper(X_W, diag([1 -1 1]) * x_c);
% 			R_ASPnL = diag([-1 1 -1]) * R_ASPnL;
% 		else
% 			[R_ASPnL, T_ASPnL] = ASPnL_wrapper(X_W, x_c);
% 		end
% 		[ER_ASPnL, ET_ASPnL, Ep_ASPnL] = errors(R_ASPnL, T_ASPnL, R_GT, T_GT, X_W);
% 		disp([R_ASPnL T_ASPnL]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ASPnL/pi,  ET_ASPnL, Ep_ASPnL);
% 	end
% 
% 	if (N_LINES >= 6)
% 		disp('LPnL_Bar_LS [R|T]');
% 		[R_LS, T_LS] = LPnL_Bar_LS_wrapper(X_W, x_c);
% 		[ER_LS, ET_LS, Ep_LS] = errors(R_LS, T_LS, R_GT, T_GT, X_W);
% 		disp([R_LS T_LS]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_LS/pi,  ET_LS, Ep_LS);
% 	end
% 
% 	if (N_LINES >= 4)
% 		disp('LPnL_Bar_ENull [R|T]');
% 		[R_ENull, T_ENull] = LPnL_Bar_ENull_wrapper(X_W, x_c);
% 		[ER_ENull, ET_ENull, Ep_ENull] = errors(R_ENull, T_ENull, R_GT, T_GT, X_W);
% 		disp([R_ENull T_ENull]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ENull/pi,  ET_ENull, Ep_ENull);
% 	end
% 
% 	if (N_LINES >= 6)
% 		disp('DLT-Lines [R|T]');
% 		[R_Lines, T_Lines] = DLT_Lines(X_W, x_c);
% 		[ER_Lines, ET_Lines, Ep_Lines] = errors(R_Lines, T_Lines, R_GT, T_GT, X_W);
% 		disp([R_Lines T_Lines]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Lines/pi,  ET_Lines, Ep_Lines);
% 	end
% 
% 	if (N_LINES >= 9)
% 		disp('DLT-Plücker-Lines [R|T]');
% 		[R_Plucker, T_Plucker] = DLT_Plucker_Lines(X_W, x_c);
% 		[ER_Plucker, ET_Plucker, Ep_Plucker] = errors(R_Plucker, T_Plucker, R_GT, T_GT, X_W);
% 		disp([R_Plucker T_Plucker]);
% 		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
% 		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Plucker/pi,  ET_Plucker, Ep_Plucker);
% 	end
% 
	if (N_LINES >= 5)
		disp('DLT-Combined-Lines [R|T]');
		[R_Combi, T_Combi] = DLT_Combined_Lines(X_W, x_c);
		[ER_Combi, ET_Combi, Ep_Combi] = errors(R_Combi, T_Combi, R_GT, T_GT, X_W);
		disp([R_Combi T_Combi]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);
	end
end

fprintf('=== Done. ===============================================\n');
