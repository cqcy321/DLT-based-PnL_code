function [ R, T ] = Ansar_wrapper( X_W, x_c )
%ANSAR_WRAPPER Camera pose estimation from line correspondences based on
%the paper Ansar, Daniilidis: Linear Pose Estimation from Points and Lines, 
%PAMI 2003.
%   This is just a wrapper that executes the implementation of Xu 
%   et al. from their paper Pose Estimation from LineCorrespondences:   
%   A Complete Analysis and A Series of Solutions, PAMI 2016.
%   (http://xuchi.weebly.com/rpnl.html).
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 4;

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
	
	%% Prepare input
	% 3D endpoints, normalize homogeneous coordinates
	for i = 1:4
		X_W(i,:) = X_W(i,:) ./ X_W(4,:);
	end
	P1_w = X_W(1:3, 1:2:end);
	P2_w = X_W(1:3, 2:2:end);
	
	% 2D endpoints, normalize homogeneous coordinates
	for i = 1:3
		x_c(i,:) = x_c(i,:) ./ x_c(3,:);
	end
	p1 = x_c(1:2, 1:2:end);
	p2 = x_c(1:2, 2:2:end);
	
	%% Call the implementation of Ansar
	[R_cw, T_cw] = Ansar(p1, p2, P1_w, P2_w);
	
	%% Convert output
	R = R_cw;
	T = R_cw \ T_cw; % R_cw\T_cw is used instead of R_cw.'*T_cw because R_cw may not be orthonormal in Ansar's method
	
	return;
end
