function S = skew3( a )
%SKEW3 Return a reverse skew 3 matrix from vector a
% Also refered as a hat operator of lie algebra so3.
%        INPUT
%  a  - 3x1 vector
%        OUTPUT
%  S  - 3x3 skew 3 matrix
S = [ 0 -a(3) a(2);... 
     a(3) 0  -a(1);...
     -a(2) a(1) 0];

end

