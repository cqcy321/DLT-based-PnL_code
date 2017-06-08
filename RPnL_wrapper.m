function [ R, T, varargout ] = RPnL_wrapper( X_W, x_c )
%RPNL_WRAPPER Camera pose estimation from line correspondences based on
%the paper of Zhang, L., et al.: Robust and Efficient Pose Estimation from Line
%Correspondences, ACCV 2012.
%   This is just a wrapper that executes the implementation of Zhang et al.
%   (http://www.mip.informatik.uni-kiel.de/tiki-index.php?page=Lilian+Zhang).
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
	
	% 2D
	x1_c = x_c(:, 1:2:end);
	x2_c = x_c(:, 2:2:end);
	
	% 3D
	for i = 1:4
		X_W(i,:) = X_W(i,:) ./ X_W(4,:);
	end
	
	X1_W  = X_W(:, 1:2:end);
	X2_W  = X_W(:, 2:2:end);
	
	V = X2_W(1:3, :) - X1_W(1:3, :);
	V  = normc(V);
	
	%% Call RPnL
	[R_RPnL, T_RPnL] = PnL(x1_c, x2_c, V, X1_W(1:3, :));
	
	%% Convert output
	R =  R_RPnL';
	T = -T_RPnL;

	return;
end

