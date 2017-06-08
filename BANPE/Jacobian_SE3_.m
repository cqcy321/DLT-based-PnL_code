function J = Jacobian_SE3_(Ls, R, t, Line_2D_unproj, K )
%JACOBIAN_SE3 此处显示有关此函数的摘要
%   此处显示详细说明

%Line_2D_unproj P * L , unprojected 2D line representation. it is the
%normal vector of the plane comprised of line and the oringinal point.
%K  projection matrix
% se3 -> tangent space of SE3. Vector g=(alfa, beta)
% alfa is the rotation vector in tangent space
% beta is the translation vector in tangent space
% J is the jacobian of size 3N * 6
    Ls_u = Ls(1:3,:);
    Ls_v = Ls(4:6,:);
    ls_u = R * Ls_u;
    ls_v = R * Ls_v;
    l = K * Line_2D_unproj;
    
    N=size(Ls,2);
    J=zeros(2*N,6); 
   i=0;
   while(i<N)
       count=i*2+1; 
       i=i+1;
       %% v  rotation  w  translation
       v = skew3(ls_u(:,i) + skew3(t) * ls_v(:,i));
       w = -skew3(ls_v(:,i));
       J_se3 = [-v w];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%speed issue
       J_proj = getJ_proj(l(:,i), K);
       J(count:count+1,:) = J_proj * J_se3;
% J(count:count+2,:) = J_se3;
   end

end

function J_proj = getJ_proj (l ,K)
    %% l is the unprojected 2D line representation 
    % K is the line projection matrix, l_image = K * l
    u = l(1);
    v = l(2);
    w = l(3);
    inv_l3 = 1.0 / l(3);
    inv_l3 = inv_l3 * inv_l3;
    k1 = K(1,:);
    k2 = K(2,:);
    k3 = K(3,:);
    J_proj = inv_l3 * [w * k1- u * k3;...
                       w * k2- v * k3];
%     J_proj = inv_l3 * [w*k1-u*k2;...
%                        w*k2-v*k3];
end
