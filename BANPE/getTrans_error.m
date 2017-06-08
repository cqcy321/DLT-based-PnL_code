function  e_t  = getTrans_error( t1,t2 )
%TRANS_ERROR Get translation error from groundtruth and estimtion
% From 2 3x1 vectors t1 and t2, the error is defined as norm(t1 - t2)

% fprintf('\nPure translation error\n');
    e_t = norm (t1 - t2);
end

