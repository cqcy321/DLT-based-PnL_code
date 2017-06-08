function [bestClassification, info] = ClassifyLinesHyst(lines, kMaxNumHyp, kMinNumHyp, kInClassResThrshldLoose, kInClassResThrshldTight, kColinearityThreshold)

% [bestClassification, info] = ClassifyLinesHyst(lines, kMaxNumHyp, kMinNumHyp, kInClassResidualThreshold)
%
% Description of the INPUT arguments:
%
% lines
% 3 x k matrix, expressing the normalized moments (on the Gaussian Sphere) 
% of the measured lines. Each column denotes the normalized moment of one
% line, and k is the number of lines which will be extracted automatically
%
% kMaxNumHyp
% Absolute maximum number of hypotheses that will be generated. The
% adaptive method (see Hartley and Zisserman, p. 120) for determining 
% # of hypotheses will stop when reaches this number
%
% kMinNumHyp
% Absolute minimum number of hypotheses that will be generated before
% adaptive method (see Hartley and Zisserman, p. 120) kicks in
%
% kInClassResidualThresholdLoose
% The threshold on cos(\theta) for detecting orthogonality in the first stage. 
% This threshold will be used to assign lines to the rough vanishing points
%
% kInClassResidualThresholdTight
% The threshold on cos(\theta) for detecting orthogonality in the second stage. 
% This threshold will be used to assign lines to the refined vanishing points
%
% Description of the OUTPUT arguments:
%
% bestClassification
% A row vectors with numbers from 0-3, corresponding to each line input in
% lines argument. 0 indicates the outliers, and 1-3 denote one of the three
% cardinal classed assigned to the line
%
% info
% A data structure containing basic debugging information. The field names
% are self explanatory
%
% Tested on Matlab (Linux) 7.9
%
% Author: Faraz Mirzaei . faraz -at- umn.edu . umn.edu/~faraz
%
% $Id: ClassifyLines.m 1209 2011-12-07 17:08:01Z faraz $

% kColinearityThreshold = cos(15*pi/180);

timeStart = tic;
% get the number of lines
kNumDetectedLines = size(lines,2);


% initialize to empty for safe return
bestQuat = [];
bestRes = [];
bestSeed = [];

% generate unique random hypotheses
kMaxID = kNumDetectedLines*(kNumDetectedLines-1)*(kNumDetectedLines-2)/6-1;
% generate more hyps to ensure enouph unique hyps available
randomValues = rand(1,ceil(kMaxNumHyp*5));
randomCompositeID = ceil(randomValues*(kMaxID-1e-8));
[~, ~, J] = unique(randomCompositeID);
randomCompositeID = randomCompositeID(unique(J));
% ensure that no more than # of unique lines are requested
kMaxNumHyp = min(length(randomCompositeID),kMaxNumHyp);
% randomCompositeID = randomCompositeID(1:kMaxNumHyp);


% initializations
numHyp = kMaxNumHyp;
hypCount = 0;
loopCount = 0;
bestClassification = zeros(1,kNumDetectedLines);
bestNumInliers = 0;
hypHistory = [];

% hypotheses generation and evaluation loop (main loop)
while(hypCount < numHyp)
    
    % incremenet the counter
    loopCount = loopCount + 1;
    % get the sample subset 
    hypSelectSample = GetCase(randomCompositeID(loopCount));
    % if the sample subset lines are co-linear, abort the loop
    lineSubset = lines(:,hypSelectSample);
    subsetInnerProduct = triu(lineSubset'*lineSubset,1);
    if max(abs(subsetInnerProduct(:))) > kColinearityThreshold
        continue
    else
        hypCount = hypCount+1;
    end
    % compute the vanishing points for this subset
    hyp = GetHypVP(lineSubset);
    % count the inliers, if more than previous, update the best hypotheses
    for i = 1:length(hyp)
        for j = 1:size(hyp(i).quat,2)
            newClassification = zeros(1,kNumDetectedLines);
            rotMatrix = quat2rot(hyp(i).quat(:,j));            
            for k = 1:kNumDetectedLines
                % vanishing point are the rows of rot matrix
                res = abs(rotMatrix*lines(:,k));
                [minRes, minClass] = min(res);
                % if one of the residuals is less than threshold, classify
                % the line accordingly
                if minRes < kInClassResThrshldLoose
                    newClassification(k) = minClass;
                end
            end
            newNumInliers = sum(newClassification ~= 0);
            
            %%{
            subsetGLine = zeros(3,newNumInliers);
            newInlierClassification = newClassification(newClassification ~= 0);
            for k = 1:newNumInliers
                subsetGLine(newInlierClassification(k),k) = 1;
            end
            
            [newQuat newRes] = EstimateVanishingPoints(lines(:,newClassification ~= 0), subsetGLine, 'relaxed');
            
            for l = 1:size(newQuat,2)
                newClassification = zeros(1,kNumDetectedLines);
                rotMatrix = quat2rot(newQuat(:,l));
                for k = 1:kNumDetectedLines
                    % vanishing point are the rows of rot matrix
                    res = abs(rotMatrix*lines(:,k));
                    [minRes, minClass] = min(res);
                    % if one of the residuals is less than threshold, classify
                    % the line accordingly
                    if minRes < kInClassResThrshldTight
                        newClassification(k) = minClass;
                    end
                end
                %%}
                
                newNumInliers = sum(newClassification ~= 0);
                
                % if more inliers are detected with this hyp, update the best hyp
                if newNumInliers > bestNumInliers
                    bestNumInliers = newNumInliers;
                    % 99% probability threshold (Hartley and Zisserman, p. 119)
                    newNumHyp = ceil(log(0.01)/log(1-(newNumInliers/kNumDetectedLines)^3));
                    %numHyp = min(kMaxNumHyp,newNumHyp);
                    numHyp = max(min(kMaxNumHyp,newNumHyp),kMinNumHyp);
                    bestClassification = newClassification;
                    bestQuat = newQuat(:,l);
                    hypHistory = [hypHistory newQuat(:,l)];
                    bestRes = newRes(:,l);
                    bestSeed = hypSelectSample;
                end
            end
        end
    end    
end

%{
% refinement the bestQuat estimate, and re-classify
bestSubsetGLine = zeros(3,bestNumInliers);
bestInlierClassification = bestClassification(bestClassification ~= 0);
for k = 1:bestNumInliers
    bestSubsetGLine(bestInlierClassification(k),k) = 1;
end
% re-estimate VP using all classified lines
[bestQuat bestRes] = EstimateVanishingPoints(lines(:,bestClassification ~= 0), bestSubsetGLine, 'relaxed');
% re-classify
newClassification = zeros(1,kNumDetectedLines);
for k = 1:kNumDetectedLines
    res = abs(quat2rot(bestQuat)*lines(:,k));
    [minRes, minClass] = min(res);
    
    % if one of the residuals is less than threshold, classify
    % the line accordingly
    if minRes < kInClassResThrshldTight
        newClassification(k) = minClass;
    end
end
bestClassification = newClassification;
%}

% create the output argument
info.hypHistory = hypHistory;
info.bestQuat = bestQuat;
info.bestRes = bestRes;
info.hypCount = hypCount;
info.bestSeed = bestSeed;
info.kInClassResidualThreshold = kInClassResThrshldTight;
info.timeClassifyLinesHyst = toc(timeStart);

