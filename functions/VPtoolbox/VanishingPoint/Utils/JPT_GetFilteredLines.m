function vsEdges = JPT_GetFilteredLines(vsEdges, fc, cc, alpha_c, kMinNumPxlInLine)

[vsEdges.point1] = vsEdges.vPointUn1;
[vsEdges.point2] = vsEdges.vPointUn2;
vsEdges = normalize_lines(vsEdges,fc,cc,alpha_c);

for i = 1:length(vsEdges)
    vsEdges(i).len = norm(vsEdges(i).vPointUn1-vsEdges(i).vPointUn2);
end

vsEdges = vsEdges([vsEdges.len] > kMinNumPxlInLine);


 