function [VP] = LoadVP_GT(im_path,name)

% $Id: LoadGroundTruth_YDB.m 1209 2011-12-07 17:08:01Z faraz $

%load(strcat(im_path,name,'LinesAndVP.mat'));
%lines_raw = lines; clear lines;
load(strcat(im_path,name,'GroundTruthVP_Orthogonal_CamParams.mat'));

VP = vp_orthogonal;
% fix the sign mismatch
VP(2,:) = -VP(2,:);
for k = 1:3
    if VP(3,k) < 0
        VP(:,k) = - VP(:,k);
    end
end
