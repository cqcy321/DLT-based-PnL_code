function [ R, T ] = Mirzaei_wrapper( X_W, x_c )
%MIRZAEI_WRAPPER Camera pose estimation from line correspondences based on
%the paper Mirzaei, Roumeliotis: Globally Optimal Pose Estimation from Line
%Correspondences, ICRA 2011.
%   This is just a wrapper that executes the implementation of Mirzaei and 
%   Roumeliotis (http://www-users.cs.umn.edu/~faraz/?p=research).
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 3;

	%% Input checks
	if (rem(size(X_W,2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(X_W,1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [X; Y; Z; W].');
	end
	
	if (size(X_W,2) ~= size(x_c,2))
		error('Number of 3D and 2D line endpoints has to be equal.');
	end;
	
	if (size(x_c,1) ~= 3)
		error('2D line endpoints have to be homogeneous coordinates - 3-tuples [x; y; w].');
	end

	N_LINES = size(X_W,2)/2;
	
	if (N_LINES < MIN_LINES)
		error(['At least ' MIN_LINES ' lines have to be supplied.']);
	end
	
	%% Directions of 3D lines
	
	% Normalize 3D line endpoints
	for i = 1:4
		X_W(i,:) = X_W(i,:) ./ X_W(4,:);
	end
	
	X1_W = X_W(:, 1:2:end);
	X2_W = X_W(:, 2:2:end);
	
	% compute normalized direction vectors
	V = X2_W(1:3,:) - X1_W(1:3,:);
	V = normc(V);
	
	%% Construct 2D lines
	x_c = diag([1; -1; 1]) * x_c;
	l(N_LINES).point1 = [];
	l(N_LINES).point2 = [];
	
	for i = 1:N_LINES
		l(i).point1 = x_c(1:2, 2*i-1)';
		l(i).point2 = x_c(1:2, 2*i  )';
	end

	% compute line moments
	fc = [1; 1];
	cc = [0; 0];
	alpha_c = 0;
	l = normalize_lines(l, fc, cc, alpha_c);

	%% Estimate camera orientation (= estimate vanishing points)
	
	if (N_LINES == 3)
		method = 'minimal';
	else
		method = 'relaxed';
	end
	
	[Q_Mirzaei, err_Mirzaei, ~] = EstimateVanishingPoints([l.nmoment], V, method);
	
	% convert quaternions to rotation matrices
	N_SOLUTIONS = size(Q_Mirzaei, 2);
	if (N_SOLUTIONS > 0)
		R_Mirzaei = zeros(3, 3, N_SOLUTIONS);
		for i = 1:N_SOLUTIONS
			R_Mirzaei(:,:,i) = quat2rot(Q_Mirzaei(:,i));
		end
	else
		R_Mirzaei = NaN(3, 3);
	end
	
	%% Estimate camera position
	% according to Eq. (20) of the paper "Mirzaei, Roumeliotis:
	% Globally Optimal Pose Estimation from Line Correspondences, ICRA 2011"
	%
	% Notation regarding Eq. (20):
	% C{w}i'.. a point on an image line ............................ xx_c
	% G{l}i .. direction vector of the 3D line in the global frame . V
	% G{m}i .. moment of the 3D line in the global frame ........... U
	% G{p}C .. camera position which is being estimated ............ T
	
	% moments of the 3D lines
	U = cross(X1_W(1:3,:), V);
	
	% compute points xx_c on the image lines
	xx1_c = [[l.pnt1_n]; ones(1,N_LINES)];
	xx2_c = [[l.pnt2_n]; ones(1,N_LINES)];
	theta = -atan((xx2_c(1,:) - xx1_c(1,:)) ./ (xx2_c(2,:) - xx1_c(2,:)));
	rho = -xx1_c(1,:) .* cos(theta) - xx1_c(2,:) .* sin(theta);
	xx_c = [-rho .* cos(theta);   -rho .* sin(theta);   ones(1,N_LINES)];

	% compute a position-solution for each orientation-solution
	T_Mirzaei = zeros(3, 1, N_SOLUTIONS);
	
	for i = 1 : N_SOLUTIONS
		left = zeros(N_LINES, 3);
		for j = 1:N_LINES
			left(j,:) =  xx_c(:,j)' * R_Mirzaei(:,:,i)' * skewSymMat(V(:,j));
		end

		right = zeros(N_LINES, 1);
		for j = 1:N_LINES
			right(j) =  -xx_c(:,j)' * R_Mirzaei(:,:,i)' * U(:,j);
		end

		warning('off', 'MATLAB:illConditionedMatrix');
		T_Mirzaei(:,:,i) = left \ right;
		warning('on', 'MATLAB:illConditionedMatrix');
	end
	
	%% Convert output
	R = zeros(3, 3, N_SOLUTIONS);
	T = zeros(3, 1, N_SOLUTIONS);
	
	for i = 1:N_SOLUTIONS
		R(:,:,i) = diag([-1; 1; -1]) * R_Mirzaei(:,:,i)';
		T(:,:,i) = - T_Mirzaei(:,:,i);
	end
	
	%% If multiple solutions exist, choose the best
	% (in front of the camera and least algebraic error)
	if (N_SOLUTIONS > 1)
		valid = false(N_SOLUTIONS, 1);
		
		for i = 1:N_SOLUTIONS
			% scene in front of camera ?
			scene_center = mean(X_W, 2);
			scene_dir = (scene_center(1:3) - T(:,:,i)) / norm(scene_center(1:3) - T(:,:,i));
			cam_dir   = R(3,:,i);
			
			test = dot(cam_dir, scene_dir);
			if (test > 0)
				valid(i) = true;
			end
		end
		
		% if no solutions are in front of the camera, consider all
		if (~any(valid))
			valid = true(N_SOLUTIONS, 1);
		end
		
		% keep only valid solutions
		R  = R(:,:,valid);
		T  = T(:,:,valid);
		
		% choose a solution with the least algebraic error
		[~, i_best] = min(err_Mirzaei);
		
		R = R(:,:,i_best);
		T = T(:,:,i_best);
	end
	
	return;
end

