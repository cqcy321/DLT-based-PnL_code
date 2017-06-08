function O = proj( I )
%PROJ 此处显示有关此函数的摘要
%   此处显示详细说明
    if (size(I,1) ~= 3)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
    end
    O = [I(1,:)./I(3,:);I(2,:)./I(3,:)];
end

