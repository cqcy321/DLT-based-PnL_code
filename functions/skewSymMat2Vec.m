function [ v ] = skewSymMat2Vec( vx_in )
%SKEWSYMMAT2VEC Returns a 3-vector from a 3x3 skew-symmetric matrix.

	if(size(vx_in, 1) ~= 3 || size(vx_in, 2) ~= 3)
		error('skewSymMat2Vec:input', 'The input has to be a 3x3 matrix.');
	end
	
	% in the case the skew-symmetric matrix is noisy, compute the nearest
	% truly skew-symmetric matrix in the sense of the Frobenius norm
	% reference: G. H. Golub: Matrix Computations
	vx = (vx_in - vx_in') / 2;
	
	v = [vx(3,2); vx(1,3); vx(2,1)];
end

