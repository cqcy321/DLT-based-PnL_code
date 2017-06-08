function [q_LS,term] = GetVPIterLS(q_init, I_moment_n, G_line, moment_COV)

% Iterative Least Squares Solver For finding Vanishing Points
% Camera is assumed to be calibrated.

% Author: faraz -at- cs.umn.edu
% $Id: GetVPIterLS.m 1260 2012-01-20 01:20:07Z faraz $

N = size(I_moment_n,2);

N_itr = 100;

if (nargin < 4)
    moment_COV(:,:,N) = eye(3);
    for k = 1:N
        moment_COV(:,:,k) = eye(3);
    end
end

% map Generated data to suitable format (line-plane)
n = zeros(3,N);
for k = 1:N
    n(:,k) = I_moment_n(:,k);%cross(I_line(:,k),I_endP1(:,k)/norm(I_endP1(:,k)));
end
e = G_line;

q_new = q_init;
C = quat2rot(q_new);

H = zeros(N,3);
res = zeros(N,1);

% set termination condition
term = false;
cost_old = inf;
lambda = 0.01;
R = zeros(N,N);

for itr = 1:N_itr

    H_old = H;
    res_old = res;

    for k = 1:N
        H(k,:) = e(:,k)'*skewsymm(C*n(:,k));
        res(k,1) = 0 - e(:,k)'*C*n(:,k);
    end

    % compute the covariance
    for k = 1:N
        R(k,k) = e(:,k)'*C*moment_COV(:,:,k)*C'*e(:,k);
    end
    
    cost_new = res'/R*res;

    if cost_new < cost_old
        % accept
        lambda = lambda/10;
        q = q_new;

        % should we terminate?
        if ( (cost_old - cost_new)/cost_old < 0.0001 || (cost_new < 0.00001) ) && ( cost_old ~= inf )
            % LS has statistically converged. terminate and set the bit.
            term = true;
            break
        else
            cost_old = cost_new;
        end
        
    else
        lambda = lambda*10;
        H = H_old;
        res = res_old;
    end

    delta = (H'/R*H+lambda*eye(3))\(H'/R*res);

    dq = [delta/2 ; 1];
    dq = dq/norm(dq);

    q_new = quat_mul(dq,q);
    C = quat2rot(q_new);

end

q_LS = q;
