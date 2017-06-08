function J = getJacobian_se3( Ls,R,t )
%GETJACOBIAN Jacobian matrix calculation function
% It returns the Jacobian from following inputs.
%   Input:
%     Ls - 6xN matrix of Plücker coordinates of 3D lines
%     R  - 3x3 rotation matrix of initial guess(eye(3) in default)
%     t  - 3x1 translation vector of initial guess(zeros(3,1) in default)
%   Output:
%     J  - 3x6 Jacobian matrix 
% ls = R * u + [t] x R * v 
% Ls = 6 * N With each column denoting a line 's Pluecker coordinates
% se3 -> tangent space of SE3. Vector g=(alfa, beta)
% alfa is the rotation vector in tangent space
% beta is the translation vector in tangent space
% J is the jacobian of size 3N * 6

%% Prequisites
    Ls_u = Ls(1:3,:);
    Ls_v = Ls(4:6,:);
    ls_u = R * Ls_u;
    ls_v = R * Ls_v;
    N=size(Ls,2);
    J=zeros(3*N,6);
    i=0;
    
%% Jacobian calculation
    while(i<N)
        count=i*3+1; 
        i=i+1;
       %% v  rotation  w  translation
        v = skew3(ls_u(:,i) + skew3(t) * ls_v(:,i));
        w = -skew3(ls_v(:,i));
        J(count:count+2,:)=[-v w];
   end

end