function [q_rep, perm, q_family] = GetSolutionFamily(q)

% get the family of VP solutions from one of them.
% result are in no specific order

% $Id: GetSolutionFamily.m 1275 2012-01-21 19:54:57Z faraz $

q_family = zeros(4,24);
ind = 0;

q_rep = [0; 0; 0; -1];

PI = eye(4);
for i = 4:-1:1
    ind = ind+1;
    q_family(:,ind) = quat_mul(PI(:,i),q);
    if q_family(4,ind) > q_rep(4)
        q_rep = q_family(:,ind);
        perm = quat2rot(PI(:,i));
    end
end

for i = 0:7
    str = dec2bin(i,3);
    sgn = (str == '0')' - (str == '1')';
    quat = [sgn*.5; .5];
    ind = ind+1;
    q_family(:,ind) = quat_mul(quat,q);
    if q_family(4,ind) > q_rep(4)
        q_rep = q_family(:,ind);
        perm = quat2rot(quat);
    end
end

sqrt2inv = 1/sqrt(2);
for i = 1:3
    for j = i+1:4
        quat = zeros(4,1);
        quat(i) = sqrt2inv;
        quat(j) = sqrt2inv;
        ind = ind+1;
        q_family(:,ind) = quat_mul(quat,q);
        if q_family(4,ind) > q_rep(4)
            q_rep = q_family(:,ind);
            perm = quat2rot(quat);
        end
        
        quat(i) = -sqrt2inv;
        ind = ind+1;
        q_family(:,ind) = quat_mul(quat,q);
        if q_family(4,ind) > q_rep(4)
            q_rep = q_family(:,ind);
            perm = quat2rot(quat);
        end
    end
end