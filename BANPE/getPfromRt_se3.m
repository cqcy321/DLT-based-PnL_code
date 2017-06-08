function [ P ] = getPfromRt_se3( R,t )
%GETPFROMRT_SE3 Get projection matrix that project a plucker coordinates toa 3D normal vector
%   Input:
%     R  - 3x3 rotation matrix
%     t  - 3x1 translation vector
%   Output:
%     P  - 3x6 projection matrix that project a line in former frame to a normal vector in new frame 

%% Input check
    if (size(R,2)~=3 |size(R,1)~=3 )
		error('Rotation matrix must be of the dimension 3*3.');
    end
    
	if (size(t,2)~=1 |size(t,1)~=3 )
		error('Transformation vector must be of the dimension 3*1.');
    end
    
%% According to defination.
    P=[R  skew3(t)*R];

end

