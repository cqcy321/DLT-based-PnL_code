%%%
% Test of the methods on the MPI datasets.
%%%


%% Settings for individual datasets - please uncomment/comment as needed.

startup;

% Comment this and uncomment the desired dataset
warning('Please comment this and uncomment a dataset of your choice in this script.'); return;

% % Timber Frame House
% folder_path = ['datasets' filesep 'MPI' filesep 'timber_frame_house' filesep];
% L_file = ['our_estimation' filesep 'complete_house.stl'];
% img_subfolder = ['used_subset' filesep];
% img_part_name = 'house_';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([1 -1  1]);
% IMG_SEQ = 200:271;
% N_LINES = 828;
% END_PT_DIST_TH = 0.2;
% REPROJ_ERR_TH = 1e-5;
% ANGLE_DIFF_TH = 10; %[°]

% % Building Blocks
% folder_path = ['datasets' filesep 'MPI' filesep 'building_blocks' filesep];
% L_file = ['our_estimation' filesep 'building_blocks.stl'];
% img_subfolder = ['used_subset' filesep];
% img_part_name = 'line_scan';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([1 -1  1]);
% IMG_SEQ = [190 191 193 197:259]; % manually filtered out images with wrongly detected 2D line segments
% N_LINES = 870;
% END_PT_DIST_TH = 0.01;
% REPROJ_ERR_TH = 1e-5;
% ANGLE_DIFF_TH = 10; %[°]

% % Street
% folder_path = ['datasets' filesep 'MPI' filesep 'street' filesep];
% L_file = 'street_3d.stl';
% img_subfolder = '';
% img_part_name = 'bridge_';
% T_FORM_3D = diag([1 1 1 1]);
% T_FORM_2D = diag([1 -1  1]);
% IMG_SEQ = 200:219;
% N_LINES = 1841;
% END_PT_DIST_TH = 0.05;
% REPROJ_ERR_TH = 1e-6;
% ANGLE_DIFF_TH = 10; %[°]

%% Extract 3D line endpoints from a .stl file
X_W = ones(4, 2*N_LINES);

f = fopen([folder_path L_file], 'r');

% skip first line
fgets(f);

for i = 1:N_LINES
	% skip two lines
	fgets(f); fgets(f);
	
	% read 1st and 2nd vertex
	vert1_line = fgets(f);
	vert2_line = fgets(f);
	[vert1, count1] = sscanf(vert1_line, '   vertex %f %f %f');
	[vert2, count2] = sscanf(vert2_line, '   vertex %f %f %f');
	if (count1 ~= 3)
		error('reading 3 values into vert1 failed');
	end
	if (count2 ~= 3)
		error('reading 3 values into vert2 failed');
	end
	
	% store vertices as line endpoints
	X_W(1:3, 2*i-1) = vert1;
	X_W(1:3, 2*i  ) = vert2;
	
	% skipt the 3rd vertex - it should be identical to the 1st one
	fgets(f);
	% skip two lines
	fgets(f); fgets(f);
end

% skip last line
fgets(f);

fclose(f);

clear f count1 count2 vert1_line vert2_line vert1 vert2 i;

%% Process each image
for img_nr = IMG_SEQ
	
	CAHV_file = [folder_path img_subfolder 'cameras' filesep img_part_name num2str(img_nr) '.cam'];
	l_file    = [folder_path img_subfolder 'lines2d' filesep img_part_name num2str(img_nr) '.lines'];
	
	fprintf('=== Processing image #%d =======================\n\n', img_nr);

	% Load CAHV camera parameters
	c = fopen(CAHV_file, 'r');
	
	% skip first 7 lines
	fgets(c); fgets(c); fgets(c); fgets(c); fgets(c); fgets(c); fgets(c);
	
	% read C
	c_line = fgets(c);
	[C_CAHV, c_count] = sscanf(c_line, '%f %f %f');
	if (c_count ~= 3)
		error('reading 3 values into C failed');
	end
	
	% skip one line
	fgets(c);
	
	% read A
	a_line = fgets(c);
	[A, a_count] = sscanf(a_line, '%f %f %f');
	if (a_count ~= 3)
		error('reading 3 values into A failed');
	end
	
	% skip one line
	fgets(c);
	
	% read H
	h_line = fgets(c);
	[H, h_count] = sscanf(h_line, '%f %f %f');
	if (h_count ~= 3)
		error('reading 3 values into H failed');
	end
	
	% skip one line
	fgets(c);
	
	% read V
	v_line = fgets(c);
	[V, v_count] = sscanf(v_line, '%f %f %f');
	if (v_count ~= 3)
		error('reading 3 values into V failed');
	end
	
	% skip two lines
	fgets(c); fgets(c);
	
	% read radial distortion
	k_line = fgets(c);
	[K, k_count] = sscanf(k_line, '%f %f');
	if (k_count ~= 2)
		error('reading 2 values into K failed');
	end
	
	% skip two lines
	fgets(c); fgets(c);
	
	% read pixel size
	p_line = fgets(c);
	[ps, p_count] = sscanf(p_line, '%f %f');
	if (p_count ~= 2)
		error('reading 2 values into pp failed');
	end
	
	% skip two lines
	fgets(c); fgets(c);
	
	% read image size
	s_line = fgets(c);
	[img_size, s_count] = sscanf(s_line, '%f %f');
	if (s_count ~= 2)
		error('reading 2 values into imgSize failed');
	end
	img_size = [img_size(2); img_size(1)];
	
	fclose(c);
	
	%% Convert CAHV camera model to KRt model
	% based on T. Pajdla: PRoVisG-D35.pdf (D.3.5 Sensor Calibration)
	% http://cmp.felk.cvut.cz/~pajdla/ProVisG-D35.pdf
	h_c = A' * H;
	v_c = A' * V;
	h_s = norm(skewSymMat(A) * H);
	v_s = norm(skewSymMat(A) * V);
	
	Hi_Vi_A = [H V A] * [1/h_s 0 0; 0 1/v_s 0; -h_c/h_s -v_c/v_s 1];
	Hi = Hi_Vi_A(:, 1);
	Vi = Hi_Vi_A(:, 2);
	
	C = [h_s 0 h_c; 0 v_s v_c; 0 0 1];
	
	principal_x = (img_size(2)-1) / 2;
	principal_y = (img_size(1)-1) / 2;
	
	C = C + [0 0 principal_x; 0 0 principal_y; 0 0 0];
	
	R_GT = [Hi Vi A]';
	T_GT = -C_CAHV;
	
	% dataset-specific transformation of 3D coordinates
	X_W = T_FORM_3D * X_W;
	R_GT = R_GT * T_FORM_3D(1:3, 1:3);
	if (det(R_GT) < 0)
		R_GT = -R_GT; % det(R_GT) must be +1!
	end
	
	% dataset-specific transformation of 2D coordinates
	C = C * T_FORM_2D;
	R_GT = inv(T_FORM_2D) * R_GT;
	if (det(R_GT) < 0)
		R_GT = -R_GT; % det(R_GT) must be +1!
	end
	
	%% Load 2D line endpoints
	l = fopen(l_file, 'r');
	x_img = fscanf(l, '%f %f %f %f\n', [2 Inf]);
	fclose(l);
	% swap x and y image coordinates
	x_img = [x_img(2,:); x_img(1,:); ones(1, size(x_img, 2))];

	x_c = inv(C) * x_img;
	for i = 1:3
		x_c(i,:) = x_c(i,:) ./ x_c(3,:);
	end
	x1_c = x_c(:, 1:2:end);
	x2_c = x_c(:, 2:2:end);
		
	%% Establish correspondences between 3D and 2D lines
	% Projecting 3D lines using ground-truth camera pose and selecting
	% correspondences based on minimal reprojection error.
	
	% project 3D lines
	D_GT = R_GT * [eye(3) T_GT]; % point displacement matrix
	x_c_gt = D_GT * X_W;
	for i = 1:3
		x_c_gt(i,:) = x_c_gt(i,:) ./ x_c_gt(3,:);
	end
	x1_c_gt = x_c_gt(:, 1:2:end);
	x2_c_gt = x_c_gt(:, 2:2:end);
	l_c_gt = cross(x1_c_gt, x2_c_gt);
	
	N_IMG_LINES = size(x1_c, 2);
	
	l_c_vec = x1_c(1:2, :) - x2_c(1:2, :);
	l_c_len = sqrt(l_c_vec(1,:).^2 + l_c_vec(2,:).^2);

	% compute reprojection errors between every projected 3D line and every image line
	Ep = zeros(N_LINES, N_IMG_LINES);
	for i = 1:N_LINES
		Ep(i, :) = reprojErrorsLines(repmat(l_c_gt(:,i), 1, N_IMG_LINES), x_c);
		Ep(i, :) = Ep(i, :) ./ l_c_len; % normalize reprojection errors by line segment lenght
	end
	
	% compute distance between endpoints of detected line segments and projected lines
	end_pt_dist = zeros(N_LINES, N_IMG_LINES);
	angle_diff = zeros(N_LINES, N_IMG_LINES);
	
	for i = 1:N_LINES
		% endpoint to line distance
		
		end_pt_dist_11 = sqrt( ...
		 (x_c(1, 1:2:end) - repmat(x_c_gt(1,2*i-1), 1, N_IMG_LINES)).^2 + ...
		 (x_c(2, 1:2:end) - repmat(x_c_gt(2,2*i-1), 1, N_IMG_LINES)).^2 ...
		);
		end_pt_dist_12 = sqrt( ...
		 (x_c(1, 1:2:end) - repmat(x_c_gt(1,2*i  ), 1, N_IMG_LINES)).^2 + ...
		 (x_c(2, 1:2:end) - repmat(x_c_gt(2,2*i  ), 1, N_IMG_LINES)).^2 ...
		);
		end_pt_dist_21 = sqrt( ...
		 (x_c(1, 2:2:end) - repmat(x_c_gt(1,2*i-1), 1, N_IMG_LINES)).^2 + ...
		 (x_c(2, 2:2:end) - repmat(x_c_gt(2,2*i-1), 1, N_IMG_LINES)).^2 ...
		);
		end_pt_dist_22 = sqrt( ...
		 (x_c(1, 2:2:end) - repmat(x_c_gt(1,2*i  ), 1, N_IMG_LINES)).^2 + ...
		 (x_c(2, 2:2:end) - repmat(x_c_gt(2,2*i  ), 1, N_IMG_LINES)).^2 ...
		);
		
		% either x1_c must be close to x1_c_gt and x2_c must be close to
		% x2_c_gt or x1_c must be close to x2_c_gt and x2_c to x1_c_gt
		end_pt_dist(i, :) = min( max(end_pt_dist_11, end_pt_dist_22), max(end_pt_dist_12, end_pt_dist_21) );
		
		% angle difference between the lines
		l_c_dir    = normc( x_c(   1:2, 1:2:end) - x_c(   1:2, 2:2:end) );
		l_c_gt_dir = normc( x_c_gt(1:2, 2*i-1)   - x_c_gt(1:2, 2*i)     );
		angle_diff(i, :) = 180*acos(abs(dot( l_c_dir, repmat(l_c_gt_dir, 1, N_IMG_LINES) )))/pi;
	end
	
	% remove correspondences with large end point distances from further processing
	Ep(end_pt_dist > END_PT_DIST_TH) = Inf;
	
	% remove correspondences with large angle difference
	Ep(angle_diff > ANGLE_DIFF_TH) = Inf;
	
	% remove correspondences with large reprojection error
	Ep(Ep > REPROJ_ERR_TH) = Inf;
	
	% find "best matching" (acc. to reprojection error) 3D line for each image line
	[min_Ep, min_ind] = min(Ep);
	
	corresp_mask_2D = ~isinf(min_Ep);
	min_ind = min_ind(corresp_mask_2D);
	
 	corresp_ind_3D = [2*min_ind-1; 2*min_ind];
	corresp_ind_3D = corresp_ind_3D(:);
	
	corresp_mask_2D = repmat(corresp_mask_2D, 2, 1);
	corresp_mask_2D = corresp_mask_2D(:);
	
	X_W_corresp = X_W(:, corresp_ind_3D);
	x_c_corresp = x_c(:, corresp_mask_2D);
	
	
	%% Camera pose estimation
	
	disp('Ground truth pose [R|T]');
	disp([R_GT T_GT]);
	fprintf('\n');
	
	if ((N_LINES >= 4) && (N_LINES <= 50))
		disp('Ansar [R|T]');
		[R_Ansar, T_Ansar] = Ansar_wrapper(X_W_corresp, x_c_corresp);
		[ER_Ansar, ET_Ansar, Ep_Ansar] = errors(R_Ansar, T_Ansar, R_GT, T_GT, X_W_corresp);
		disp([R_Ansar T_Ansar]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Ansar/pi,  ET_Ansar, Ep_Ansar);
	end

	if (N_LINES >= 3)
		disp('Mirzaei [R|T]');
		[R_Mirzaei, T_Mirzaei] = Mirzaei_wrapper(X_W_corresp, x_c_corresp);
		[ER_Mirzaei, ET_Mirzaei, Ep_Mirzaei] = errors(R_Mirzaei, T_Mirzaei, R_GT, T_GT, X_W_corresp);
		disp([R_Mirzaei T_Mirzaei]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Mirzaei/pi,  ET_Mirzaei, Ep_Mirzaei);
	end

	if (N_LINES >= 4)
		disp('RPnL [R|T]');
		[R_RPnL, T_RPnL] = RPnL_wrapper(X_W_corresp, x_c_corresp);
		[ER_RPnL, ET_RPnL, Ep_RPnL] = errors(R_RPnL, T_RPnL, R_GT, T_GT, X_W_corresp);
		disp([R_RPnL T_RPnL]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_RPnL/pi,  ET_RPnL, Ep_RPnL);
	end

	if (N_LINES >= 3)
		disp('ASPnL [R|T]');
		[R_ASPnL, T_ASPnL] = ASPnL_wrapper(X_W_corresp, x_c_corresp);
		[ER_ASPnL, ET_ASPnL, Ep_ASPnL] = errors(R_ASPnL, T_ASPnL, R_GT, T_GT, X_W_corresp);
		disp([R_ASPnL T_ASPnL]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ASPnL/pi,  ET_ASPnL, Ep_ASPnL);
	end

	if (N_LINES >= 6)
		disp('LPnL_Bar_LS [R|T]');
		[R_LS, T_LS] = LPnL_Bar_LS_wrapper(X_W_corresp, x_c_corresp);
		[ER_LS, ET_LS, Ep_LS] = errors(R_LS, T_LS, R_GT, T_GT, X_W_corresp);
		disp([R_LS T_LS]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_LS/pi,  ET_LS, Ep_LS);
	end

	if (N_LINES >= 4)
		disp('LPnL_Bar_ENull [R|T]');
		[R_ENull, T_ENull] = LPnL_Bar_ENull_wrapper(X_W_corresp, x_c_corresp);
		[ER_ENull, ET_ENull, Ep_ENull] = errors(R_ENull, T_ENull, R_GT, T_GT, X_W_corresp);
		disp([R_ENull T_ENull]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_ENull/pi,  ET_ENull, Ep_ENull);
	end

	if (N_LINES >= 6)
		disp('DLT-Lines [R|T]');
		[R_Lines, T_Lines] = DLT_Lines(X_W_corresp, x_c_corresp);
		[ER_Lines, ET_Lines, Ep_Lines] = errors(R_Lines, T_Lines, R_GT, T_GT, X_W_corresp);
		disp([R_Lines T_Lines]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Lines/pi,  ET_Lines, Ep_Lines);
	end

	if (N_LINES >= 9)
		disp('DLT-Plücker-Lines [R|T]');
		[R_Plucker, T_Plucker] = DLT_Plucker_Lines(X_W_corresp, x_c_corresp);
		[ER_Plucker, ET_Plucker, Ep_Plucker] = errors(R_Plucker, T_Plucker, R_GT, T_GT, X_W_corresp);
		disp([R_Plucker T_Plucker]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Plucker/pi,  ET_Plucker, Ep_Plucker);
	end
	
	if (N_LINES >= 5)
		disp('DLT-Combined-Lines [R|T]');
		[R_Combi, T_Combi] = DLT_Combined_Lines(X_W_corresp, x_c_corresp);
		[ER_Combi, ET_Combi, Ep_Combi] = errors(R_Combi, T_Combi, R_GT, T_GT, X_W_corresp);
		disp([R_Combi T_Combi]);
		fprintf('\tOrient.err[°]   Pos.err[]  Reproj.err[]\n');
		fprintf('\t%13f %11f %13.4e\n\n\n', 180*ER_Combi/pi,  ET_Combi, Ep_Combi);
	end
	
	% pause();
end

fprintf('=== Done. ===============================================\n');
