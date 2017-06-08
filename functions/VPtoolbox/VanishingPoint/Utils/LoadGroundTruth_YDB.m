function [lines dir VP] = LoadGroundTruth_YDB(im_path,name,fc,cc,alpha_c)

% $Id: LoadGroundTruth_YDB.m 1231 2012-01-07 00:39:07Z faraz $

load(strcat(im_path,name,'LinesAndVP.mat'));
lines_raw = lines; clear lines;
load(strcat(im_path,name,'GroundTruthVP_Orthogonal_CamParams.mat'));


N_ln = length(lines_raw)/2;
lines(N_ln).point1 = [];
lines(N_ln).point2 = [];
dir = zeros(3,N_ln);
for k = 1:N_ln
    lines(k).point1 = lines_raw(2*k-1,:);
    lines(k).point2 = lines_raw(2*k,:);
    dir(vp_association(k),k) = 1;
end

lines = normalize_lines(lines,fc,cc,alpha_c);

VP = vp_orthogonal;
% fix the sign mismatch
VP(2,:) = -VP(2,:);
for k = 1:3
    if VP(3,k) < 0
        VP(:,k) = - VP(:,k);
    end
end
