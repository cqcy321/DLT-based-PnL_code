function [ cp ] = closestPoint2SetOfLines( x, varargin )
%CLOSESTPOINT2SETOFLINES Computes the closest point to the set of lines in
%space.
% Minimizes the sum of SQUARED distances to the lines defined by endPts
% (not the sum of distances) because it can be computed in closed form,
% although the squared distance is more prone to outliers. The lines might
% have weights assigned as a column vector in the second argument.
%
%  x - (D+1)x(2N) matrix of homogeneous line endpoints [x; y; z; ...; w],
%      where D is the dimensions of space (2D, 3D) and N is the number of lines.
%  w - Nx1 column vector of weights for the lines.
%
% Source: K. Aftab, R. Hartley, J. Trumpf: "Lq-Closest-Point to Affine Subspaces
% using the Generalized Weiszfeld Algorithm", Eq. (12)

%% Input checks
	if (rem(size(x, 2), 2))
		error('Number of line endpoints has to be an even number.');
	end
	
	if (size(x,1) < 3)
		error('The line endpoints have to be homogeneous coordinates - so at least 3-tuples [x; y; w] for the 2D case.');
	end
	
	N_LINES = size(x, 2)/2;
	DIM = size(x, 1) - 1;
	
	if (N_LINES < 2)
		error('At least 2 lines has to be supplied.');
	end
	
	switch(nargin)
		case 1
			w = ones(N_LINES, 1); % default unit weights w
		case 2
			w = varargin{1}; % vector of weights w
			if(size(w, 1) ~= N_LINES)
				error('Number of weights has to be equal to the number of lines.');
			end
		otherwise
			error('Too many input arguments. Maximal 2 arguments accepted.');
	end
	
	
	%% Normalize the homogeneous coordinates of endpoints
	for i = 1:(DIM+1)
		x(i,:) = x(i,:) ./ x(end,:);
	end

	%% Algorithm
	e = x(1:DIM, 2:2:end) - x(1:DIM, 1:2:end);
	e = normc(e); % orthonormal base of the subspace, i.e. the unit direction vector of current line
	A = repmat(e(:), 1, DIM) .* kron(e', ones(DIM, 1));
	M = repmat(eye(DIM), N_LINES, 1) - A;
	C = x(1:DIM, 1:2:end);

	M = sqrt(kron(w, ones(DIM  , DIM))) .* M;
	c = sqrt(kron(w, ones(DIM, 1))) .* dot(M, kron(C', ones(DIM, 1)), 2);
		
	% compute the closest point
	cp = (M'*M) \ (M' * c);
	
	return;

end

