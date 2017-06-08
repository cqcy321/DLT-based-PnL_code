function [ R_refined, t_refined ] = LBA_se3(ls, Ls,R,t )
%LBA Local Bundle Adjustment method to get the pose estimation.
% Use projection of 3D lines in Plücker coordinates to new frame to estimate pose.
%   Input:
%     Ls - 6xN matrix of Plücker coordinates of 3D lines
%     ls - 3xN matrix of homogeneous coordinates of 2D lines
%     R  - 3x3 rotation matrix of initial guess(eye(3) in default)
%     t  - 3x1 translation vector of initial guess(zeros(3,1) in default)
%   Output:
% R_refined - 3x3 calculated rotation matrix 
% t_refined - 3x1 calculated translation vector

    N  = size(ls,2);
    R1 = R;
    t1 = t;
    P  = getPfromRt_se3(R1,t1);%3x6 Projection matrix
    iteration = 0;
    b_minus_f=ls-P*Ls;
    b_minus_f=b_minus_f(:);
    err = norm (b_minus_f);
%     fprintf('Iteration %d, error is %x\n',iteration,err);

%% Iterations to approximate the pose
    while(iteration<20)
        J = getJacobian_se3(Ls, R1, t1);
        delta = (J'*J) \ (J' * b_minus_f);
        [R1,t1] = update_se3(R1, t1, delta);
        P = getPfromRt_se3(R1,t1);
        iteration=iteration+1;
        b_minus_f=ls-P*Ls;
        err=norm(b_minus_f);
        b_minus_f=b_minus_f(:);
%         fprintf('Iteration %d, error is %x\n',iteration,err);
    end
    
%% Output the final result    
    R_refined=R1;
    t_refined=t1;
end

