function [R_,t_]=update_se3(R1,t1,delta)
%UPDATE Use se3 vector delta to update the rotation matrix R1 and translatation vector t1
%Update measurement matrix.
%   Input:
%     R1 - 3x3 matrix of original rotation 
%     t1 - 3x1 vector of original translation
%   delta- 6*1 vector in se3 space, where first 3 in the rotation space and latter 3 in translation space.
%   Output:
%     R_ - 3x3 matrix of updated rotation 
%     t_ - 3x1 vector of updated translation 
	if (size(delta,2)~=1 |size(delta,1)~=6 )
		error('Transformation vector must be of the dimension 3*1.');
    end

    omega = delta(1:3);
    Omega = skew3(omega);
    upsilon = delta(4:6);
    theta = norm(omega);
    if (theta <0.00001)
        R = eye(3) + Omega + Omega * Omega;
        V = R;
    else
        Omega2 = Omega * Omega;
        R = eye(3) + sin(theta) / theta * Omega + (1 - cos(theta))/ (theta * theta) * Omega2;
        V = eye(3) + (theta - sin(theta)) / (theta^3) * Omega2 + (1 - cos(theta))/ (theta * theta) * Omega;
    end
    t = V * upsilon;
    R_ = R * R1;
    t_ = R * t1 + t;
end

