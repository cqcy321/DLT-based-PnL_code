function ind = GetCase(id)

kEqualityTol =  1e-12;

% Given an id, compute a valid combination of the 3 image index.
% it is responsibility of the user to ensure that the input argument
% does not exceed the available number of images.
% Example: GetCase(0) = [1 2 3] (this is the lowest combination id)
% Example: Given 5 lines, there will be 5*4*3/6 = 10 valid combinations,
% their corresponding index will be from 0 to 9.
% GetCase(9) = [5 4 3]

% Author: Faraz Mirzaei, University of Minnesota
% Contact: faraz -at- umn.edu
% Copyright (c) 2011, 2012 The Regents of The University of Minnesota

% $Id: GetCase.m 1239 2012-01-09 16:30:16Z faraz $

ind = zeros(1,3);
id = id + 1;

% get first id
% solve nchoosek(l,3) = id, and get an estimate for l
lCandidates = roots([1 -3 2 -6*id]);
lCandidates = real(lCandidates(imag(lCandidates) < eps));
lCandidates = lCandidates(lCandidates>0);
assert(~isempty(lCandidates),'candidates for generation of sample triplets impossible!');
l = max(lCandidates);

if abs(l - round(l)) < kEqualityTol
    ind = round([l l-1 l-2]);
else
    l = floor(l);
    ind(1) = l+1;
    id = id -nchoosek(l,3);
    
    
    lCandidates = roots([1 -1 -2*id]);
    lCandidates = real(lCandidates(imag(lCandidates) < eps));
    lCandidates = lCandidates(lCandidates>0);
    assert(~isempty(lCandidates),'candidates for generation of sample triplets impossible!');
    l = max(lCandidates);
    
    if abs(l - round(l)) < kEqualityTol
        ind(2) = round(l);
        ind(3) = round(l-1);
    else
        l = floor(l);        
        ind(2) = l+1;
        ind(3) = id - nchoosek(l,2);
    end
end