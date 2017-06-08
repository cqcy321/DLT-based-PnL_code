function [ R ] = interpRotation( R1, R2, k )
%INTERPROTATION Interpolates between 2 rotations by a factor of k (0-1).
% Rotations are represented by rotation matrices. The factor of 0 means R1
% is returned, the factor of 1 means R2 is returned.

	log_R_diff = logm(R2' * R1);
	
	if (~isreal(log_R_diff))
		log_R_diff = sign(real(log_R_diff)) .* abs(log_R_diff);
	end

	R_cor = expm((1-k) * log_R_diff);
	
	R = R2 * R_cor;
end

