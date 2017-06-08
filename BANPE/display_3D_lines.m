function o = display_3D_lines( points, points_est )
%DISPLAY_3D_LINES Display the 3D lines
%   Detailed explanation goes here

	if (size(points,1) < 3)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
    end
	
    N = size(points, 2) / 2;
    i = 0;
    figure;
    while(i<N)
        plot3( points(1,2*i+1:2 * i+2),points(2,2*i+1:2 * i+2) , points(3,2*i+1:2 * i+2),'g');
        plot3(  points_est(1,2*i+1:2 * i+2) ,points_est(2,2*i+1:2 * i+2),points_est(3,2*i+1:2 * i+2),'r');
        hold on;
        i = i + 1;
    end
    legend('Estimated Lines','Real Lines','location','best');
    o = true;
    view(3);
    xlabel('x');
    ylabel('y');
    zlabel('z');

end

