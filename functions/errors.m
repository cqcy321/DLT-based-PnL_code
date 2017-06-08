function [ ER, ET, Ep ] = errors( R, T, R_GT, T_GT, X_W )
%ERRORS Rotation, translation and reprojection error for given camera pose.
%   R    - 3x3xN ... set of N rotation matrices
%   T    - 3x1xN ... set of N translation vectors
%   R_GT - 3x3 ... ground-truth rotation matrix
%   T_GT - 3x1 ... ground-truth translation vector
%   X_W  - 4x2n ... endpoints of 3D line segments in homogeneous coordinates [W; Y; Z; W]
%
%   ER - Rotation error
%   ET - Translation error
%   Ep - Reprojection error

	N_SOLUTIONS = size(R, 3);
	
	ER = zeros(N_SOLUTIONS, 1);
	ET = zeros(N_SOLUTIONS, 1);
	Ep = zeros(N_SOLUTIONS, 1);

	% project 3D endpoints using ground-truth camera pose
	TM_GT = R_GT * [eye(3) T_GT];
	x_c_GT = TM_GT * X_W;
	l_c_GT = cross(x_c_GT(:, 1:2:end), x_c_GT(:, 2:2:end));
	
	
	for i = N_SOLUTIONS:-1:1 % finish with best solution
		
		% if any entry in R or T is Inf, set also all error measures to Inf
		if (any(any(isinf(R(:,:,i)))) || any(any(isinf(T(:,:,i)))))
			ER(i) = Inf;
			ET(i) = Inf;
			Ep(i) = Inf;
			continue;
		end
		
		% project 3D endpoints using estimated camera pose
		TM = R(:,:,i) * [eye(3) T(:,:,i)];
		x_c = TM * X_W;
		
		warning ('off','all');
		% temporarily disable warnings since rot2aa functions produces warnings
		% about small angles, which mean low orientation error
		[~, Theta] = rot2aa(R_GT' * R(:,:,i));
		warning ('on','all');
		
		% Rotation error
		if (~isnan(Theta))
			ER(i) = abs(Theta);
		else
			ER(i) = Inf;
		end
		
		% Translation error
		ET(i) = norm(T_GT - T(:,:,i));
		
		% Reprojection error
		Ep(i) = reprojErrorLines(l_c_GT, x_c);
		
	end

	return;
end

