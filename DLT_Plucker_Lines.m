function [ R, T ] = DLT_Plucker_Lines( X_W, x_c )
%DLT_PLUCKER_LINES Camera pose estimation from line correspondences using
%the method from the paper: Přibyl, B., Zemčík, P. and Čadík, M.: Camera Pose
%Estimation from Lines using Plücker Coordinates, BMVC 2015,
%http://dx.doi.org/10.5244/C.29.45
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 9;

%% Input checks
	if (rem(size(X_W, 2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(X_W, 1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [X; Y; Z; W].');
	end
	
	if (size(X_W, 2) ~= size(x_c, 2))
		error('Number of 3D and 2D line endpoints has to be equal.');
	end;
	
	if (size(x_c, 1) ~= 3)
		error('2D line endpoints have to be homogeneous coordinates - 3-tuples [x; y; w].');
	end
	
	N_LINES = size(X_W,2)/2;
	
	if (N_LINES < MIN_LINES)
		error(['At least ' MIN_LINES ' lines have to be supplied.']);
	end
	
	
 %% Create Plücker representation of 3D lines
	L_W = pluckerLines(X_W);
	
	%% Prenormalization of 3D lines
	
	% Normalize each 3D line s.t. norm(V)=sqrt(3)
	for i = 1:N_LINES
		V = L_W(4:6,i);
		L_W(:,i) = sqrt(3) * L_W(:,i) ./ norm(V);
	end

	% Translation: translate 3D lines s.t. the closest point to them is the Origin
	T_prenorm = closestPoint2SetOfLines(X_W);
	DT = [eye(3) skewSymMat(-T_prenorm); zeros(3) eye(3)]; % line displacement matrix
	L_W_T = DT * L_W;

	% Anisotropic scaling: scale 3D lines s.t. their U and V parts have the same
	% average magnitude
	U_abs_mean = mean(     abs( L_W_T(1:3, :) ), 2);
	V_abs_mean = mean(mean(abs( L_W_T(4:6, :) ) ) );
	S_prenorm = V_abs_mean ./ U_abs_mean;
	DS  = [diag(S_prenorm) zeros(3); zeros(3) eye(3)]; % line scaling matrix

	% Combine translation and scaling
	DSM = DS * DT; % line similarity matrix
	
	% Apply all prenormalizing transformations at once
	L_W = DSM * L_W;
	
	%% Construct 2D line equations from projected endpoints
	l_c = cross(x_c(:, 1:2:end), x_c(:, 2:2:end));
	
	%% Pre-normalization of 2D lines - treat them as 2D points

	% "Translation": "translate" lines so that their centroid is at the Origin
	t_prenorm = mean([l_c(1,:) ./ l_c(3,:); l_c(2,:) ./ l_c(3,:)], 2);
	dt = [eye(2) -t_prenorm; 0 0 1];
	l_c_t = dt * l_c;

	% "Scaling": "scale" lines anisotropically
	s_prenorm = 1 ./ mean(abs([l_c_t(1,:) ./ l_c_t(3,:); l_c_t(2,:) ./ l_c_t(3,:)]), 2);
	ds = diag([s_prenorm;  1]);

	% Combine "translation" and "scaling"
	dsm = ds * dt;
	
	% Apply all prenormalizing transformations at once
	l_c = dsm * l_c;
	
	%% Construct the measurement matrix

	M1 = kron( ...
		[1 1 1 1 1 1], ...
		[ ...
			l_c(3,:)'    zeros(N_LINES,1)   -l_c(1,:)'; ...
			zeros(N_LINES,1)  l_c(3,:)'     -l_c(2,:)'  ...
		] ...
	);
	M2 = kron([L_W L_W]', [1 1 1]);
	M  = M1 .* M2;
	
	%% Linear estimation of the line projection matrix
	if (size(M,1) < size(M,2))
		[~, ~, V] = svd(M);
	else
		[~, ~, V] = svd(M, 'econ');
	end
	
	% Form a 3x6 estimate of the line projection matrix from the last right singular vector
	P_e = reshape( V(:,end), 3, 6 );
	
	%% Post-transformation reverting the prenormalizing transformations
	P_e = dsm \ P_e;  % revert "translation" and "scaling" of 2D lines
	
	% post-transformation un-doing the pre-normalizing transformations of 3D lines
	P_e = P_e * DS; % revert scaling of 3D lines
	% translation of 3D lines is reverted _after_ camera position is estimated (better accuracy)
	
	%% Extract parameters from the line projection matrix
	
	% Algorithm 1 from the paper: scale estimation
	[U_R, Sigma_R, V_R] = svd(P_e(:, 1:3));
	det_UV = det(U_R * V_R');
	s = det_UV / mean(diag(Sigma_R));
	
	% Algoritm 3 from the paper: decomposition of an essential matrix = R.[t]x
	% inpired by Tsai and Huang: Uniqueness and estimation of 3D motion..., PAMI 1984
	[U_P, Sigma_P, V_P] = svd(s * P_e(:, 4:6));

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
		R = R_A;
		T = T_A - T_prenorm;
	else
		R = R_B;
		T = T_B - T_prenorm;
	end
		
	return;
end
