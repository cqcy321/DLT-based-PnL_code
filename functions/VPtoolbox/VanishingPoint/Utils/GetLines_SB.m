function lines = GetLines_SB(imagePath)

% $Id: GetLines_SB.m 1231 2012-01-07 00:39:07Z faraz $

lineFinder = '/home/faraz/devel/linedetection/linefinder  ';
unix([lineFinder, imagePath, '> linebuffer']);
load linebuffer;

numLines = size(linebuffer,1);

lines(numLines).point1 = [];
lines(numLines).point2 = [];

for k = 1:numLines
    
    res = linebuffer(k,1:3)*[linebuffer(k,4:5),1]';
    pnt = linebuffer(k,4:5) - linebuffer(k,1:2)*res + 1;
    % switch the coordinates
    lines(k).point1(1) = pnt(2);
    lines(k).point1(2) = pnt(1);    
    
    res = linebuffer(k,1:3)*[linebuffer(k,6:7),1]';
    pnt = linebuffer(k,6:7) - linebuffer(k,1:2)*res + 1;
    % switch the coordinates
    lines(k).point2(1) = pnt(2);
    lines(k).point2(2) = pnt(1);    

    
end
