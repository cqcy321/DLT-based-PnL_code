function lines = GetLines_PK(im,N_ln,out_path,draw_flag)

% $Id: GetLines_PK.m 1263 2012-01-20 02:00:42Z faraz $

% minimum number of points (pixels) to detect a valid line
kMinNumPoints = 70;

% Peter Kovesi's line segment detector

if ~exist('draw_flag','var')
    draw_flag = false;
end

% Find edges using the Canny operator with hysteresis thresholds of 0.1
% and 0.2 with smoothing parameter sigma set to 1.
edgeim = edge(im,'canny', [0.1 0.2], 1);

if draw_flag == true
    figure(1), imshow(edgeim); % truesize(1)
end

% Link edge pixels together into lists of sequential edge points, one
% list for each edge contour.  Discard contours less than 10 pixels long.
[edgelist, labelededgeim] = edgelink(edgeim, 100);

% Display the labeled edge image with random colours for each
% distinct edge in figure 2
if draw_flag == true
    drawedgelist(edgelist, size(im), 1, 'rand', 2); axis off
end

% Fit line segments to the edgelists
tol = 2;         % Line segments are fitted with maximum deviation from
% original edge of 2 pixels.
seglist = lineseg(edgelist, tol);

% Draw the fitted line segments stored in seglist in figure window 3 with
% a linewidth of 2 and random colours
if draw_flag == true
    drawedgelist(seglist, size(im), 2, 'rand', 3); axis off
end

if exist('out_path','var')
    save(strcat(out_path,'PK_LINES.mat'),'seglist')
end

%
contour_no = length(seglist);
len = zeros(1,contour_no*10);
ids = zeros(2,contour_no*10);
ind = 1;
for i = 1:contour_no
    part_no = length(seglist{i});
    for j = 1:part_no-1
        len(ind) = norm(seglist{i}(j+1,:)-seglist{i}(j,:));
        ids(:,ind) = [i;j];
        ind = ind + 1;
    end
end
    
[val,sorted_ind] = sort(len,'descend');
%N_ln = 40;

% if we have very few lines, ignore N_ln and generate half # of segments
% N_ln = min(N_ln,round(ind/2));
N_ln = min(N_ln,find( val>kMinNumPoints ,1,'last'));
%min_len = val(N_ln);
lines(N_ln).point1 = [];
lines(N_ln).point2 = [];
for k = 1:N_ln
    contour_id = ids(1,sorted_ind(k));
    segment_id = ids(2,sorted_ind(k));
    lines(k).point1 = seglist{contour_id}(segment_id,2:-1:1);
    lines(k).point2 = seglist{contour_id}(segment_id+1,2:-1:1);
end
