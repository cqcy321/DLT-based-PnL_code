function [ R, T ] = DLT_Combined_Lines( X_W, x_c )
%DLT_COMBINED_LINES Camera pose estimation from line correspondences using
% the DLT-Combined-Lines method (Přibyl, B., Zemčík, P. and Čadík, M.:
% Pose Estimation from Line Correspondences using Direct Linear Transformation).
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 5;

	%% Input checks
	if (rem(size(X_W, 2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(X_W, 1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [X; Y; Z; W].');
	end
	
	if (size(X_W,2) ~= size(x_c, 2))
		error('Number of 3D and 2D line endpoints has to be equal.');
	end;
	
	if (size(x_c, 1) ~= 3)
		error('2D line endpoints have to be homogeneous coordinates - 3-tuples [x; y; w].');
	end

	N_LINES = size(X_W, 2)/2;
	
	if (N_LINES < MIN_LINES)
		error(['At least ' MIN_LINES ' lines have to be supplied.']);
	end
	
	
	%% Create Plücker representation of 3D lines
	L_W = pluckerLines(X_W);
	
	%% Prenormalization of the 3D points/lines
	
	% Normalize each 3D point s.t. X_W(4)=1 and 
	for i = 1:4
		X_W(i,:) = X_W(i,:) ./ X_W(4,:);
	end
	
	% Normalize each 3D line s.t. norm(V)=sqrt(3)
	for i = 1:N_LINES
		V = L_W(4:6,i);
		L_W(:,i) = sqrt(3) * L_W(:,i) ./ norm(V);
	end
	
	% Translation 1: translate 3D points/lines s.t. centroid of points is at the Origin
	T_prenorm_1 = mean( X_W(1:3, :), 2 );
	
	D1_pts = [eye(3) -T_prenorm_1; 0 0 0 1];
	D1_lns = [eye(3) skewSymMat(-T_prenorm_1); zeros(3) eye(3)];
	
	X_W_t1 = D1_pts * X_W;
	L_W_t1 = D1_lns * L_W;
	
	% Translation 2: s.t. mean([X_W_t1(1:3) L_t1(1:3)]) = 0
	a = 4 * N_LINES;
	b = sum(L_W_t1 (1,:)) + sum(X_W_t1 (1,:));
	c = sum(L_W_t1 (2,:)) + sum(X_W_t1 (2,:));
	d = sum(L_W_t1 (3,:)) + sum(X_W_t1 (3,:));
	e = sum(L_W_t1 (4,:));
	f = sum(L_W_t1 (5,:));
	g = sum(L_W_t1 (6,:));
	
	T_prenorm_2_X =	-(a^2*b + b*e^2 - a*c*g + a*d*f + c*e*f + d*e*g) / (a*(a^2 + e^2 + f^2 + g^2));
	T_prenorm_2_Y = -(a^2*c + c*f^2 + a*b*g - a*d*e + b*e*f + d*f*g) / (a*(a^2 + e^2 + f^2 + g^2));
	T_prenorm_2_Z = -(a^2*d + d*g^2 - a*b*f + a*c*e + b*e*g + c*f*g) / (a*(a^2 + e^2 + f^2 + g^2));
	T_prenorm_2 = [T_prenorm_2_X; T_prenorm_2_Y; T_prenorm_2_Z];
	
	% Combine and apply Translation 1 and Translation 2
	T_prenorm = T_prenorm_1 + T_prenorm_2;
	DT_pts = [eye(3) -T_prenorm(1:3); 0 0 0 1];                % point displacement matrix
	DT_lns = [eye(3) skewSymMat(-T_prenorm); zeros(3) eye(3)]; % line displacement matrix
	X_W_T = DT_pts * X_W;
	L_W_T = DT_lns * L_W;
	
	% Anisotropic scaling: scale 3D points/lines s.t. mag([X u1]) = mag([Y u2]) = mag([Z u3]) = mag([W v])
	U_abs_mean = mean(     abs( L_W_T(1:3, :) ), 2);
	V_abs_mean = mean(mean(abs( L_W_T(4:6, :) ) ) );
	X_abs_mean = mean(abs( X_W_T ), 2);
	
	S_prenorm = (X_abs_mean(4) + V_abs_mean) ./ (X_abs_mean(1:3) + U_abs_mean);

	DS_pts   = diag([S_prenorm; 1]); % point scaling matrix
	DS_lns   = [ ...                % line scaling matrix
		diag(S_prenorm) zeros(3); ...
		    zeros(3)     eye(3)   ...
	];
	DS_combi = [ ...                % combined scaling matrix
		diag(S_prenorm)  zeros(3,4); ...
    0     0     0    1   0 0 0 ; ...
		     zeros(3,4)      eye(3)  ...
	];
	
	% Combine translation and scaling
	DSM_pts = DS_pts * DT_pts; % point similarity matrix
	DSM_lns = DS_lns * DT_lns; % line similarity matrix

	% Apply all prenormalizing transformations at once
	X_W = DSM_pts * X_W;
	L_W = DSM_lns * L_W;
	
	%% Construct 2D lines from endpoints
	l_c = cross(x_c(:, 1:2:end), x_c(:, 2:2:end));
	
	% No prenormalization of 2D lines
	
	%% Construct the combined measurement matrix
	
	X1_W = X_W(:, 1:2:end);
	X2_W = X_W(:, 2:2:end);

	% measurement matrix for point-line correspondences
	M1_pts = kron([1 1 1 1], [l_c l_c]');
	M2_pts = kron([X1_W X2_W]', [1 1 1]);
	M_pts  = M1_pts .* M2_pts;
	
	% measurement matrix for line-line correspondences
	M1_lns = kron( ...
		[1 1 1 1 1 1], ...
		[ ...
			l_c(3,:)'    zeros(N_LINES,1)   -l_c(1,:)'; ...
			zeros(N_LINES,1)  l_c(3,:)'     -l_c(2,:)'  ...
		] ...
	);
	M2_lns = kron([L_W L_W]', [1 1 1]);
	M_lns  = M1_lns .* M2_lns;
	
	% Normalize point and line measurement matrices acoording to the sums of squares
	M_pts_SS = sum(sum(M_pts.^2));
	M_lns_SS = sum(sum(M_lns.^2 ));
	
	M_pts = M_pts / sqrt(M_pts_SS);
	M_lns = M_lns / sqrt(M_lns_SS);
	
	% Combine point / line measurement matrices into combined measurement matrix
	M = [ ...
		            M_pts                  zeros(2*N_LINES, 9); ...
		M_lns(:, 1:9)  zeros(2*N_LINES, 3)    M_lns(:, 10:18)   ...
	];
	
	
	%% Linear estimation of the combined projection matrix
	if (size(M,1) < size(M,2))
		[~, ~, V_M] = svd(M);
	else
		[~, ~, V_M] = svd(M, 'econ');
	end

	% Form a 3x7 estimate of the combined projection matrix from the last right singular vector
	P_e = reshape( V_M(:,end), 3, 7 );
	
	%% Post-transformation reverting the prenormalizing 3D transformations
	P_e = P_e * DS_combi; % revert scaling
	% translation is reverted _after_ camera position is estimated (better accuracy)
	
	%% Extract parameters from the combined projection matrix
	
	% Divide combined projection matrix into submatrices
	P1_e = P_e(:, 1:3);
	P2_e = P_e(:, 4);
	P3_e = P_e(:, 5:7);
	
	% Algorithms 1 + 2 from the paper: scale estimation + R1 orthogonalization
	[U_R, Sigma_R, V_R] = svd(P1_e);
	det_UV = det(U_R * V_R');
	s = det_UV / mean(diag(Sigma_R));
	R1 = U_R * diag([det_UV det_UV det_UV])  * V_R';
	
	% T2 computation
	T2 = s * R1' * P2_e  - T_prenorm;
		
	% Algoritm 3 from the paper: decomposition of an essential matrix = R.[t]x
	% inpired by Tsai and Huang: Uniqueness and estimation of 3D motion..., PAMI 1984
	[U_P, Sigma_P, V_P] = svd(s * P3_e);

	Z = [0  1  0; -1  0  0; 0  0  0];
	W = [0 -1  0;  1  0  0; 0  0  1];
	q = (Sigma_P(1,1) + Sigma_P(2,2)) / 2;
	
	% 2 possible solutions A/B
	det_A = det(U_P * W  * V_P');
	det_B = det(U_P * W' * V_P');
	
	R_A = U_P * W  *  diag([1 1 det_A])  * V_P';
	R_B = U_P * W' *  diag([1 1 det_B])  * V_P';
	
	Tx_A = q * V_P * Z  * V_P';
	Tx_B = q * V_P * Z' * V_P';
	
	T_A = skewSymMat2Vec(Tx_A);
	T_B = skewSymMat2Vec(Tx_B);
	
	test = dot( R_A(3,:),  -T_A/norm(-T_A) );
	
	if (test > 0)
		R3 = R_A;
		T3 = T_A - T_prenorm;
	else
		R3 = R_B;
		T3 = T_B - T_prenorm;
	end
	
		
	%% Output
	k = 0.7;
	
	R = interpRotation(R3, R1, k);
	T = k * T2  +  (1-k) * T3;
	
	return;

end
