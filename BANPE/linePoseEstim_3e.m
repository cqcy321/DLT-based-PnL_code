function [ R_e, t_e ] = linePoseEstim_3e( line_3D_end_pts, line_2D_end_pts, C_GT ,R_estim, t_estim)
%LINEPOSEESTIM_3E Pose estimation from line correspondences.
%		INPUT: 
%   line_3D_end_pts - 4x(2N) matrix of 3D line start- and end-points in homogeneous coordinates [x; y; z; w]
%   line_2D_end_pts - 3x(2N) matrix of 2D line start- and end-points in homogeneous coordinates [x; y; w] in the image plane
%        C_GT       - 3x3 camera matrix. (refered as K)
%		OUTPUT:
%		R_e - 3x3 rotation matrix
%		t_e - 3x1 translation vector
%
	
	% input checks
	
	if (size(line_3D_end_pts,1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
	end
	
	if (size(line_3D_end_pts,2) ~= size(line_2D_end_pts,2))
		error('Number of 3D and 2D line endpoints has to be equal.');
	end;
	
	if (size(line_2D_end_pts,1) ~= 3)
		error('2D line endpoints have to be homogeneous coordinates - 3-tuples [x; y; w].');
	end

	NLINES = size(line_3D_end_pts,2)/2;
    K = inv(C_GT');
    
    	%% Create Plücker representation of 3D lines 6xN plucker matrix made of vectors
	lines_3D = pluckerLines_eric(line_3D_end_pts);
% 	P_GT  = getPfromRt_se3(R_estim, t_estim);
%     lines_2D_GT = P_GT * lines_3D;
%     lines_2D_GT = proj(lines_2D_GT);

	%% Construct 2D line equations from projected endpoints 3xN line 2D vector
	lines_2D =proj( cross(line_2D_end_pts(:, 1:2:end), line_2D_end_pts(:, 2:2:end)));
    
    
    
%     prec =lines_2D_GT(:) ./lines_2D(:);
%     lines_2D =[ lines_2D(1,:) ./ lines_2D(3,:);  lines_2D(2,:) ./ lines_2D(3,:)]
%     lines_2D = lines_2D_GT;

    %% Initial Guess
%     q = [rand rand rand rand];
    R_ = eye(3);
    t_ = 0.1 * [rand;rand;rand];
    t_ = zeros(3,1);
    R_ = R_estim;
    t_ = t_estim;
%     R_ =quat2dcm(q);
    
%     t_ =[rand rand rand]';
%% Lines Bundle Adjustments
    N = NLINES;
    P = getPfromRt_se3(R_ , t_);%3x6 Projection matrix
    iteration = 0;
    err = inf;
    thre = 5e-2;
    Line_2D_unproj = P * lines_3D  ;
    b_minus_f = lines_2D - proj(K * Line_2D_unproj);
    b_minus_f = b_minus_f(:);
    err=norm(b_minus_f);
%     fprintf('Iteration %d, error is %x\n',iteration,err);
    while(iteration < 20)
        J = Jacobian_SE3_(lines_3D, R_, t_, Line_2D_unproj, K );
        if(rcond(J'*J)<10^-15)
            break
        end
        delta=inv(J'*J)*(J'*b_minus_f);
        [R_,t_]=update_se3(R_,t_,delta);
        P = getPfromRt_se3(R_,t_);
        iteration=iteration+1;
        Line_2D_unproj = P * lines_3D;  
        b_minus_f = lines_2D - proj(K * Line_2D_unproj);
        b_minus_f = b_minus_f(:);
        err=norm(b_minus_f);
%         fprintf('Iteration %d, error is %x\n',iteration,err);

    end
    
% end 
    
%         fprintf('Iteration %d, error is %x\n',iteration,err);
R_e = R_;
t_e = t_;
end
