function [ is_inlier_best ] = MLESAC( X_W, x_c, PnL_func, sample_size, probab, max_iter_init, Ep_th )
%MLESAC A MLESAC loop (a modification of RANSAC) calling PnL_func to generate
%hypotheses about camera pose.
%   Returns a true/false vector which determines the inlying line
%   correspondences.

	% Coded according to "Calibrated Camera Resectioning Using P3P in RANSAC
	% Scheme" by Petr Matousek, http://cmp.felk.cvut.cz/cmp/courses/TDV/2010W/labs/p3pransac.pdf
	
	N_LINES = size(X_W, 2)/2;
	iter = 0;
	max_iter = max_iter_init;
	support_best = 0;
	is_inlier_best = false(N_LINES, 1);
	rng('shuffle', 'twister');
	
	while(iter < max_iter)
		
		iter = iter + 1;
		
		% randomly select nlines_min correspondences
		idx = randperm(N_LINES, sample_size);
		idx = [2*idx-1; 2*idx];
		
		X_W_sample = X_W(:, idx(:));
		x_c_sample = x_c(:, idx(:));	

		% generate a hypothesis about the camera pose
		[R_hyp, T_hyp] = PnL_func(X_W_sample, x_c_sample);
		
		% for each hypothesis: project lines, compute the set of inliers and support
		N_HYP = size(R_hyp, 3);
		for h = 1:N_HYP
			
			% project lines
			TM_hyp = R_hyp(:,:,h) * [eye(3) T_hyp(:,:,h)];

			% select 2D points whose 3D lines are in front of camera (i.e. both 3D endpoints are in front of camera)
			x_c_proj_hyp = TM_hyp * X_W;
			is_front_x = (x_c_proj_hyp(3,:) < 0);
			is_front_x = reshape(is_front_x, 2, N_LINES);
			is_front_l = all(is_front_x);
			x_c_proj_hyp_front = x_c_proj_hyp(:, [is_front_l; is_front_l]);
			
			x_c_hyp_front = x_c(:, [is_front_l; is_front_l]);
			
			% construct 2D lines
			x1_c_proj_hyp_front = x_c_proj_hyp_front(:, 1:2:end);
			x2_c_proj_hyp_front = x_c_proj_hyp_front(:, 2:2:end);
			l_c_proj_hyp_front = cross(x1_c_proj_hyp_front, x2_c_proj_hyp_front);
			
			% identify inliners based on reprojection error
			Epl = reprojErrorsLines( l_c_proj_hyp_front, x_c_hyp_front );
			is_inlier = (Epl < Ep_th);

			% support approximating a ML estimator
			support = sum(1 - (Epl(is_inlier).^2 / Ep_th^2));
			
			if(support > support_best)
				% update the best hypothesis
				support_best   = support;
				is_inlier_best = is_inlier;
				
				% update the stopping criterion
				epsilon = 1 - (sum(is_inlier) / N_LINES);
				max_iter = log(1 - probab) / log(1 - (1 - epsilon)^sample_size);
				if(max_iter > max_iter_init)
					max_iter = max_iter_init;
				end
			end
			
		end
	end % while

	return;
end
