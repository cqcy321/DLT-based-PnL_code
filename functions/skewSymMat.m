function [ vx ] = skewSymMat( v )
%SKEWSYMMAT Returns a 3x3 skew-symmetric matrix for a 3-vector.

	if(length(v) ~= 3)
		error('skewSymMat:input', 'The input has to be a 3-vector.');
	end

	vx = [ ... 
		   0    -v(3)  v(2); ...
		 v(3)    0    -v(1); ...
		-v(2)  v(1)    0   ...
	];

end

