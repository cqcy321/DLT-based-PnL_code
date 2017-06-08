function [ E_theta, E_t, P_theta, P_t ] = Comparative_exp( NLINES, R, t, cam, sig )
%COMPARATIVE_EXP 
% Author : Yu Cao, Beihang University 5/23/2017
% We make comparative experiments to compare the error by our proposed
% method (prefixed E) with the BMVC 2015 method (prefixed P).
% Several random lines are generated to calculate the pose from different
% method.

%         INPUT
%  NLINES -- Number of lines.
%    R    -- Groundtruth rotation matrix.
%    t    -- Groundtruth translation matrix.
%   sig   -- Image noise in 0-mean gaussian distribution of variance sig.
%         OUTPUT
% E_theta -- our mehtod's rotation error in degree.
%   E_t   -- our mehtod's translation error in meter.
% P_theta -- Pribyl's mehtod's rotation error in degree.
%   P_t   -- Pribyl's mehtod's translation error in meter.
%

%% First we generate random 2N 3D points to compose N lines
    i = 0;
    End_points_3D = zeros(4,2*NLINES);
    len = 2 * NLINES;
while(i < len)
    i = i + 1;
    point = 10*[rand rand rand 0.1]';
    End_points_3D(:,i) = point;
end
    
%% 3D lines in plucker coordinates
    Lines=pluckerLines_eric(End_points_3D);
    
%% 3D transform to end points in camera coordinates    
    T = [R skew3(t) * R];
    T_motion = [R t];
    End_points_2D_ = T_motion * End_points_3D;
    
  


%% Pose estimation in our method
    End_points_2D = cam * End_points_2D_; %%%%Where question lays
    End_points_2D = [End_points_2D(1, :) ./ End_points_2D(3, :);  End_points_2D(2, :) ./ End_points_2D(3, :); ones(1,size(End_points_2D,2))];  
        End_points_2D = End_points_2D + [normrnd(0, sig, 2, size(End_points_2D,2)); zeros(1, size(End_points_2D,2)) ];

    
    %% Pose estimation Czech
    End_points_2D_1 = cam \ End_points_2D;
    tic
    [R_estim, t_estim] = DLT_Lines( End_points_3D, End_points_2D_1);
    toc
    t_estim = R_estim * t_estim;
    
%     R1=eye(3);  %% initial guess from whatever
%     t1=[0;0;0 ];   
    tic
    [R_, T_]= linePoseEstim_3e(End_points_3D, End_points_2D,cam, R_estim, t_estim);
    toc
%% Display the pose parameters.
    disp('Real pose [R|t]');
    disp([R t]);

    disp('Estimated pose [R|t]');
    disp([R_estim t_estim]);

    disp('Optimized pose [R|t]');
    disp([R_  T_]);
    
%% 3D view of the reprojected lines and the groundtruth lines
    T_motion = [R_ T_];
    End_points_2D_est = T_motion * End_points_3D;
%     End_points_2D_est = [End_points_2D_est(1, :) ./ End_points_2D_est(3, :);  End_points_2D_est(2, :) ./ End_points_2D_est(3, :); ones(1,size(End_points_2D_est,2))];  
    display_3D_lines(End_points_2D_, End_points_2D_est);

%% Calculate the error terms.
    E_theta = getAngle_error(R_, R);
    E_t = getTrans_error(t,T_);
    P_theta = getAngle_error(R_estim, R);
    P_t = getTrans_error(t,t_estim);
end

