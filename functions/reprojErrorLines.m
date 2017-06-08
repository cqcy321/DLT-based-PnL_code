function [ Ep, varargout ] = reprojErrorLines( l_c, x_c )
%REPROJERRORLINES Computes reprojection error of lines / line segments.
%   This function computes a reprojection error of line segments to lines,
%   based on: C.J.Taylor and D.J.Kriegman: "Structure and Motion from Line
%   Segments in Multiple Images".
%
%		The return value is a scalar (number) - a sum of individual
%		reprojection errors between the pairs l_c(i) - x_c(2i-1 : 2i).
%   The alternative second output is the vector of individual reprojection
%   errors.

	%% Input checks
	if(size(l_c, 1) ~= 3)
		error('reprojErrorLines:argin', 'The 1st input argument has to be of size 3xN.');
	end

	if((size(x_c, 1) ~= 3) || rem(size(x_c, 2), 2))
		error('reprojErrorLines:argin', 'The 2nd input argument has to be of size 3xN, where N is even.');
	end
	
	if((2 * size(l_c, 2)) ~= size(x_c, 2))
		error('reprojErrorLines:argin', 'The 2nd argument has to contain twice as much elements as the 1st argument (endpoints of line segments).');
	end
	
	
	%% Compute the reprojection error
	
	% normalize x_c to [x; y; 1]
	for i = 1:3
		x_c(i,:) = x_c(i,:) ./ x_c(3,:);
	end
	
	N_LINES = size(l_c, 2);

	% reprojection error for individual lines
	Epl = zeros(N_LINES, 1);
	
	for i = 1:N_LINES
		m = l_c(:, i);
		A = x_c(:, 2*i-1:2*i)';	
		line_len = sqrt((A(1,1) - A(2,1))^2 + (A(1,2) - A(2,2))^2);
		B = (line_len / (3 * (m(1)^2 + m(2)^2))) * [1.0  0.5; 0.5  1.0];
		Epl(i) = abs(m' * (A' * B * A) * m);
	end
	
	
	%% Output
	Ep = mean(Epl);
	
	if(nargout > 1)
		varargout{1} = Epl;
	end
	
	return;

end
