function lines_3D = pluckerLines_eric(line_3D_end_pts)
%PLUCKERLINES Plücker coordinates of 3D lines from its endpoints.
%   INPUT:
%     line_3D_end_pts - 4x(2N) matrix of 3D line start- and end-points in homogeneous coordinates [x; y; z; w]
%   OUTPUT:
%      plucker_lines - 6xN matrix of Plücker coordinates of 3D lines

%% Input checks
	if (rem(size(line_3D_end_pts,2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(line_3D_end_pts,1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
    end
    
%% Prenormalize procedure, currently not neccesary.
    if(0)
		% proper pre-normalization (translation and isotropic scaling) of 3D
		% Plucker lines cannot be done here as Plucker homogeneous 6-tuples can
		% have 0 coordinates (~ 5D homog points at infinity). Translation to
		% the Origin must suffice here.
	
		% translate lines so that the closest point to them is the Origin
		shift_3D = closestPoint2SetOfLines(line_3D_end_pts);
		pre_tform_3D_pts = [eye(3) -shift_3D(1:3); 0 0 0 1];
	else
		shift_3D = [0; 0; 0];
		pre_tform_3D_pts = eye(4);
    end	
	line_3D_end_pts = pre_tform_3D_pts * line_3D_end_pts;
    
%% Plucker coordinates calculation.
    N = size(line_3D_end_pts, 2) / 2;
    lines_3D = zeros(6, N);   
    i=0;
   while(i<N)
       count = i*2+1; 
       i = i+1;
       a = line_3D_end_pts(1:3,count);
       aw = line_3D_end_pts(4,count);
       b = line_3D_end_pts(1:3,count+1);
       bw = line_3D_end_pts(4,count+1);
       u = cross(a, b);
       v = aw * b - bw * a;
       lines_3D(:,i) = [u; v];
   end
end

