function [maxRMS minRMS totalRMS] = GetVPDistance(VP, lines, VP_GT, dir, fc, cc)

% $Id: GetVPDistance.m 1263 2012-01-20 02:00:42Z faraz $

% number of lines
N_ln = length(lines);

K = diag([fc(1) fc(2) 1]);
K(1,3) = cc(1);
K(2,3) = cc(2);

lines = normalize_lines(lines,fc,cc,0);
%[~, class] = min(abs(VP'*[lines.nmoment]));

% associate estimated VPs with GT VPs
[val, class] = max(abs(VP'*VP_GT),[],1);
% set class to zero if less than 3VPs are estimated
if size(VP,2) < 3
    [~, indS] = sort(val,'descend');
    class(indS(size(VP,2)+1:3)) = 0;
end
% classify lines
allClass = class*dir;

pixelVP = K*VP;

sumDistSquare = zeros(1,3);
for i = 1:N_ln
    
    % get the class
    %[~, class] = min(abs(VP'*lines(i).nmoment));
    j = allClass(i);
    
    if j ~= 0
        % get the center
        cent = (lines(i).point1+lines(i).point1)'/2;
        
        % get the line passing through the VP
        lineDir = cross([cent; 1], pixelVP(:,j));
        
        % compute the distance to two ends
        sumDistSquare(j) = sumDistSquare(j) + ...
            ([lines(i).point1, 1]*lineDir)^2/(lineDir(1:2)'*lineDir(1:2)) + ...
            ([lines(i).point2, 1]*lineDir)^2/(lineDir(1:2)'*lineDir(1:2));
    end
end

totalSum = 0;
totalNum = 0;
for j = 1:size(VP,2)
    distRMS(j) = sqrt(sumDistSquare(j)/sum(allClass==j));
    totalSum = totalSum + sumDistSquare(j);
    totalNum = totalNum + sum(allClass==j);
end

if length(distRMS) < 3
    maxRMS = inf;
else
    maxRMS = max(distRMS);
end

minRMS = min(distRMS);

totalRMS = sqrt(totalSum/totalNum);

