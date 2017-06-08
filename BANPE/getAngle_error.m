function e_theta = getAngle_error(  R_GT,R_estim )
% Get the error of rotation from the measurement and the ground truth in degrees.
    qq = dcm2quat(R_GT*R_estim');
    e_theta = acos(qq(1)) * 180 / pi;
    % fprintf('\nPure rotation error\n');theta*180/3.1415926%%% should be *2.....DOUBLED error is the true error.
end

