function diff = DiffToSlnFmly(q1,q2)

% $Id: DiffToSlnFmly.m 1263 2012-01-20 02:00:42Z faraz $

[~, ~, q_all] = GetSolutionFamily(q1);

resQuat = quat_mul_batch(q_all, quat_inv(q2));
[~, id] = max(resQuat(4,:));
diff = 2*(resQuat(1:3,id));
