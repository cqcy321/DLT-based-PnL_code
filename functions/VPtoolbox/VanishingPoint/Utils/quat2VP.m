function VP = quat2VP(q)

% $Id: quat2VP.m 1263 2012-01-20 02:00:42Z faraz $

VP = quat2rot(q)';

for k = 1:3
    if VP(3,k) < 0
        VP(:,k) = -VP(:,k);
    end
end

% [~, ind] = sort(VP(3,:));
% 
% VP = VP(:,ind);
