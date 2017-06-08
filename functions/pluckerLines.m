function [ L ] = pluckerLines( X_W )
%PLUCKERLINES Plücker coordinates of 3D lines from its endpoints.
%   INPUT:
%     X_W - 4x(2N) matrix of 3D line start- and end-points in
%           homogeneous coordinates [x; y; z; w]
%   OUTPUT:
%      plucker_lines - 6xN matrix of Plücker coordinates of 3D lines

	%% Input checks
	if (rem(size(X_W, 2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(X_W, 1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
	end
	
	%% Method
	X1_W = X_W(:, 1:2:end);
	X2_W = X_W(:, 2:2:end);
	
	plucker_mat1 = repmat(X1_W(:), 1, 4) .* kron(X2_W', ones(4, 1));
	plucker_mat2 = repmat(X2_W(:), 1, 4) .* kron(X1_W', ones(4, 1));
	
	plucker_mat = plucker_mat1 - plucker_mat2;
	
	U = [plucker_mat(2:4:end, 3) plucker_mat(3:4:end, 1) plucker_mat(1:4:end, 2)]';
	V = [plucker_mat(4:4:end, 1) plucker_mat(4:4:end, 2) plucker_mat(4:4:end, 3)]';
	
	L = [U; V];
	return;
end

