function lines = GetLines_PK_Cashed(N_ln,in_path, kMinNumPoints)

% kMinNumPoints
% minimum number of points (pixels) to detect a valid line
%
% $Id: GetLines_PK_Cashed.m 1231 2012-01-07 00:39:07Z faraz $

load(strcat(in_path,'PK_LINES.mat'))

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
%N_ln = min(N_ln,round(ind/2));
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
