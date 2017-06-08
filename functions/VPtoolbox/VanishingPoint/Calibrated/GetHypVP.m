function hyp = GetHypVP(nmoments)

% Generate Hypotheses for vanishing points and focal length based on the
% given four normalized moments. 

% Author: Faraz Mirzaei, University of Minnesota
% Contact: faraz -at- umn.edu
% Copyright (c) 2011, 2012 The Regents of The University of Minnesota

% $Id: GetHypVP.m 1244 2012-01-10 02:00:49Z faraz $


% maximum residual tolerance above which the solution is rejected directly.
% note that the residual should be zero in most cases, since we are using
% minimal number of lines to determine the orientation (and VPs)
% however, to allow for imaginary solutions of minimal problem, we set 
% the threshold to somwhat higher value
kMaxResidualTol = 1e-3;

% different configurations
G_line{1} = eye(3);
G_line{2} = zeros(3); G_line{2}([1 4 8]) = 1;
G_line{3} = zeros(3); G_line{3}([1 5 7]) = 1;
G_line{4} = zeros(3); G_line{4}([2 4 7]) = 1;

accumulator(8).quat = [];
accumulator(8).G_line = [];
count = 0;
% the following loop can be simplified to improve its speed. 
% refer to GetHypVPFoc to see how to simplify.
for k = 1:4
    [quat, res] = EstimateVanishingPoints(nmoments, G_line{k}, 'minimal');
    new_hyp.quat = quat(:,res < kMaxResidualTol);
    %GetVP_kd(nmoments,training_id, G_line{k},ORTH_THER);
    if ~isempty(new_hyp.quat)
        new_hyp.G_line = G_line{k};        
        accumulator(count+1:count+length(new_hyp)) = new_hyp;
        count = count + length(new_hyp);
    end
end

hyp = accumulator(1:count);