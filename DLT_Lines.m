function [ R, T ] = DLT_Lines( X_W, x_c )
%DLT_LINES Camera pose estimation from line correspondences using the
%method from the book Hartley and Zisserman: Multiple View Geometry in
%Computer Vision, p. 180.
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 6;

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

	N_LINES = size(X_W, 2)/2;
	
	if (N_LINES < MIN_LINES)
		error(['At least ' MIN_LINES ' lines have to be supplied.']);
	end
	

	%% Prenormalization of 3D points
	
	% Normalize each 3D point s.t. X_W(4)=1 and 
	for i = 1:4
		X_W(i,:) = X_W(i,:) ./ X_W(4,:);
	end
	
	% Translation: translate 3D points s.t. centroid of points is at the Origin
	T_prenorm = mean( X_W(1:3, :), 2 );
	DT = [eye(3) -T_prenorm(1:3); 0 0 0 1]; % point displacement matrix
	X_W_T = DT * X_W;

	% Anisotropic scaling: scale 3D points/lines s.t. their mean distance to the
	% Origin is sqrt(3)
	S_prenorm = 1 ./ mean(abs( X_W_T(1:3, :) ), 2);
	DS = diag([S_prenorm; 1]); % point scaling matrix

	% Combine translation and scaling
	DSM = DS * DT; % point similarity matrix
	
	% Apply all prenormalizing transformations at once
	X_W = DSM * X_W;	
	
	
	%% Construct 2D line equations from projected endpoints
	l_c = cross(x_c(:, 1:2:end), x_c(:, 2:2:end));
		
	%% Prenormalization of 2D lines - treat them as 2D points

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
	
	X1_W = X_W(:, 1:2:end);
	X2_W = X_W(:, 2:2:end);
	
	M1 = kron([1 1 1 1], [l_c l_c]');
	M2 = kron([X1_W X2_W]', [1 1 1]);
	M = M1 .* M2;
	
	%% Linear estimation of the point projection matrix
	if (size(M,1) < size(M,2))
		[~, ~, V] = svd(M);
	else
		[~, ~, V] = svd(M, 'econ');
	end
	
	% Form a 3x4 estimate of the point projection matrix from the last right singular vector
	P_e = reshape( V(:,end), 3, 4 );
	
	%% Post-transformation reverting the prenormalizing transformations
	P_e = dsm' * P_e; % revert "translation" and "scaling" of 2D lines

	% post-transformation un-doing the pre-normalizing transformations of 3D endpoints
	P_e = P_e * DS; % revert scaling of 3D points
	% translation of 3D points is reverted _after_ camera position is estimated (better accuracy)
	
	%% Extract parameters from the point projection matrix
	
	% Algorithm 1 + 2 from the paper: scale estimation + R orthogonalization
	[U_R, Sigma_R, V_R] = svd(P_e(:, 1:3));
	det_UV = det(U_R * V_R');
	s = det_UV / mean(diag(Sigma_R));
	
	R = U_R  *  diag([det_UV det_UV det_UV])  *  V_R';
	T = s * R' * P_e(:, 4)  -  T_prenorm;

	return;
end
