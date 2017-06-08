function [ C ] = getCameraMatrix( varargin )
%GETCAMERAMATRIX Returns a 3x3 camera matrix based on input parameters.

	% process parameters
	switch nargin
		case 3
			aspect     = 1.0;
			skew_angle = 0.0;
		case 4
			aspect     = varargin{4};
			skew_angle = 0.0;
		case 5
			aspect     = varargin{4};
			skew_angle = varargin{5};
		otherwise
			error('getCameraMatrix expects 3 - 5 input parameters, %d given.', nargin);
	end
	
	focal       = varargin{1};
	principal_x = varargin{2};
	principal_y = varargin{3};

	% build the matrix
	skew = aspect * focal * tan(skew_angle);

	C = [ ...
		focal      skew      principal_x; ...
			0    aspect*focal  principal_y; ...
			0          0            1       ...	     
	];

end

