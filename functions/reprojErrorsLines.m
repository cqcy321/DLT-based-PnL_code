function [ Epl ] = reprojErrorsLines( l_c, x_c )
%REPROJERRORSLINES Computes reprojection errors of lines / line segments.
%   This function computes reprojection errors for every pair line segment -
%   - line, based on: C.J.Taylor and D.J.Kriegman: "Structure and Motion 
%   from Line Segments in Multiple Images".
%
%		The return value is a vector of reprojection errors between the pairs
%		l_c(i) - x_c(2i-1 : 2i).

	%% Input checks
	if(size(l_c, 1) ~= 3)
		error('reprojErrorsLines:argin', 'The 1st input argument has to be of size 3xN.');
	end

	if((size(x_c, 1) ~= 3) || rem(size(x_c, 2), 2))
		error('reprojErrorsLines:argin', 'The 2nd input argument has to be of size 3xN, where N is even.');
	end
	
	if((2 * size(l_c, 2)) ~= size(x_c, 2))
		error('reprojErrorsLines:argin', 'The 2nd argument has to contain twice as much elements as the 1st argument (endpoints of line segments).');
	end
	
	
	%% Compute the reprojection errors
	
	% normalize x_c to [x; y; 1]
	for i = 1:3
		x_c(i,:) = x_c(i,:) ./ x_c(3,:);
	end

	x1_c_1 = x_c(1, 1:2:end);
	x1_c_2 = x_c(2, 1:2:end);
	x2_c_1 = x_c(1, 2:2:end);
	x2_c_2 = x_c(2, 2:2:end);

	line_len = sqrt((x1_c_1 - x2_c_1).^2  + (x1_c_2 - x2_c_2).^2);

	l_c_1 = l_c(1,:);
	l_c_2 = l_c(2,:);
	l_c_3 = l_c(3,:);

	h1 = (l_c_1 .* x1_c_1 + l_c_2 .* x1_c_2 + l_c_3) ./ sqrt(l_c_1.^2 + l_c_2.^2);
	h2 = (l_c_1 .* x2_c_1 + l_c_2 .* x2_c_2 + l_c_3) ./ sqrt(l_c_1.^2 + l_c_2.^2);

	Epl = (line_len / 3) .* (h1.^2 + (h1 .* h2) + h2.^2);
	
	return;
end

