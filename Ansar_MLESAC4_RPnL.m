function [ R, T ] = Ansar_MLESAC4_RPnL( X_W, x_c, probab, max_iter, Ep_th)
%ANSAR_MLESAC4_RPNL Camera pose estimation from line correspondences using
% the Ansar method plugged into a MLESAC loop. The final solution is
% computed using the RPnL method, because Ansar cannot handle many lines.
%
%   X_W - 4x(2N) matrix of 3D line endpoints [X; Y; Z; W]
%   x_c - 3x(2N) matrix of 2D line endpoints [x; y; w]
%   ... where N = number of line segments.
%
%   The 3D world coordinate system is right-handed.
%   The 3D camera coordinate system is right-handed: X-right, Y-up, Z-back.

	MIN_LINES = 4;

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
	

	%% Identify inliers using MLESAC	
	is_inlier = MLESAC(X_W, x_c, @Ansar_wrapper, MIN_LINES, probab, max_iter, Ep_th);

	%% Compute the final solution using all inliers
	if (size(is_inlier, 1) > size(is_inlier, 2))
		is_inlier = is_inlier';
	end
	are_inliers = [is_inlier; is_inlier];
	
	X_W_inliers = X_W(:, are_inliers(:));
	x_c_inliers = x_c(:, are_inliers(:));
	
	if (sum(is_inlier) < MIN_LINES)
		% not enough inliers
		R = NaN(3,3);
		T = NaN(3,1);
	else
		% enough inliers
		[R, T] = RPnL_wrapper(X_W_inliers, x_c_inliers);
	end

	return;
end

