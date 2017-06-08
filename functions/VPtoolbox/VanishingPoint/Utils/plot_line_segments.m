function xy_long = plot_line_segments(I,lines, class)

% plot line segments overlaid on the image and output the endpoints of the
% longest line segment
%
% $Id: plot_line_segments.m 1375 2012-07-23 02:47:47Z faraz $

if nargin < 3 
    class = zeros(size(lines));
end

colors{1} = [236 73 20]/236;
colors{2} = [.7 .78 1];
colors{3} = [0 1 0];
colors{4} = [255 255 0]/255;
colors{5} = [255 255 0]/255;
colors{6} = [255 255 0]/255;
colors{7} = [255 255 0]/255;
colors{8} = [255 255 0]/255;


figure, imshow(I,'Border','tight'), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',3,'Color',colors{class(k)+1});
   %text((xy(1,1)+xy(2,1))/2,(xy(1,2)+xy(2,2))/2,strcat(num2str(k)),'FontSize',20,'Color','g')

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

