function [bestQuat, probVgL] = ClassifyLinesExpMax(lines, bestQuat)

% initialize prior VP priorities
% global q_DLS
% $Id: ClassifyLinesExpMax.m 1262 2012-01-20 01:55:00Z faraz $

sigma = 0.01;
itr = 0;

N_ln = size(lines,2);

probV = [0.23; 0.23; 0.23; 0.31];

probVgL = zeros(4,N_ln);

changeInVP = inf;


while (changeInVP > 1e-4)

    itr = itr+1;
    if(itr > 100)
        bestQuat = [];
        return
    end

    
    %vp = quat2rot(bestQuat)';
    vp = quat2VP(bestQuat);
    
    for i = 1:N_ln
        probLgV = zeros(4,1);
        for k = 1:3
            probLgV(k) = exp(-0.5*(lines(:,i)'*vp(:,k)/sigma)^2)/(2*pi*sqrt(2*pi*sigma^2));
        end
        probLgV(4) = 1/(4*pi^2);
        
        normalizeFactor = probV'*probLgV;
        
        probVgL(:,i) = probV.*probLgV/normalizeFactor;
    end
    
    % adjustment of prior. This is suggested by Antone00
    probV = sum(probVgL,2)/size(probVgL,2);
    probV = probV/sum(probV);
    
    momentsAggregate = [lines lines lines];
    gLineAggregate = [[sqrt(probVgL(1,:)); zeros(2,N_ln)],...
        [zeros(1,N_ln); sqrt(probVgL(2,:)); zeros(1,N_ln)],...
        [zeros(2,N_ln); sqrt(probVgL(3,:))]];
    
    % opts.conditioning = false;
    % opts.outputFormat = 2;
    % opts.kActionFunctionIndex = 2;
    % opts.kNormalizationTol = 1e-16;
    % opts.kPolyResidualTol = inf;
    % opts.kSolutionInClassTol = .95;
    % opts.kMulMatrixCondToAbort = 1e14;
    % opts.Ne = 16;
    
    [quat, res] = EstimateVanishingPoints(momentsAggregate, gLineAggregate, 'relaxed');
    
    if size(quat,2) == 0
        bestQuat = [];
        return
    end
    
    [~, ~, q_all] = GetSolutionFamily(quat(:,1));
    
    resQuat = quat_mul_batch(q_all, quat_inv(bestQuat));
    [~, id] = max(resQuat(4,:));
    changeInVP = 2*norm(resQuat(1:3,id));
    
    bestQuat = quat(:,1);
    
    %resQuat = quat_mul_batch(q_all, quat_inv(q_DLS));
    %[~, id] = max(resQuat(4,:));
    %devToGTH = 2*norm(resQuat(1:3,id));
    
    %[changeInVP, minInd] = min(absChangeQual);
    %[changeInVP devToGTH]
    
end

for i = 1:N_ln
    probLgV = zeros(4,1);
    for k = 1:3
        probLgV(k) = exp(-0.5*(lines(:,i)'*vp(:,k)/sigma)^2)/(2*pi*sqrt(2*pi*sigma^2));
    end
    probLgV(4) = 1/(4*pi^2);
    
    normalizeFactor = probV'*probLgV;
    
    probVgL(:,i) = probV.*probLgV/normalizeFactor;
end


% probVgL(1:3,:) = abs(perm)*probVgL(1:3,:);

