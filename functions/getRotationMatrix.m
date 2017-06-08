function [ R ] = getRotationMatrix( varargin )
%GETROTATIONMATRIX Returns a 3x3 rotation matrix based on input rotation angles.

	% camera orientation (rotation/Euler angles -> rotation matrix)

	% process parameters
	switch nargin
		case 3
			rot_order = 'zyx';
		case 4
			rot_order = varargin{4};
		otherwise
			error('getRotationMatrix expects 3 - 4 input parameters, %d given.', nargin);
	end
	
	Alpha = varargin{1};
	Beta  = varargin{2};
	Gamma = varargin{3};

	if (~checkRotOrder(rot_order))
		error('getRotationmatrix: the 4th argument has to contain exactly 3 characters, ''x'', ''y'', and ''z''. The ordering is arbitrary.');
	end


	% build individual axis rotation matrices
	R_x = [ ...
		1       0          0;     ...
		0  cos(Alpha) sin(Alpha); ...
		0 -sin(Alpha) cos(Alpha)  ...
	];

	R_y = [ ...
		cos(Beta) 0 -sin(Beta); ...
				0     1      0;     ...
		sin(Beta) 0  cos(Beta)  ...
	];

	R_z = [ ...
		cos(Gamma) sin(Gamma) 0;  ...
	 -sin(Gamma) cos(Gamma) 0;  ...
				 0          0     1   ...
	];

	
	% Evaluate order of rotations and build the final R matrix
	R = eye(3);
	
	for i = 1:3
		rot_char = rot_order(i);
		switch (rot_char)
			case 'x'
				R_i = R_x;
			case 'y'
				R_i = R_y;
			case 'z'
				R_i = R_z;
		end
		R = R_i * R;
	end
	
	return;

end


function [ valid ] = checkRotOrder( rot_order )
	if (length(rot_order) ~= 3)
		valid = false;
		return;
	end
	
	x_pos = strfind(rot_order, 'x');
	y_pos = strfind(rot_order, 'y');
	z_pos = strfind(rot_order, 'z');
	
	if (length(x_pos) ~= 1 || length(y_pos) ~= 1 || length(z_pos) ~= 1)
		valid = false;
		return;
	end
	
	valid = true;
	return;
end

