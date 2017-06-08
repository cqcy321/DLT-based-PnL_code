startup;
NLINES = 200;
Sigma = 1;
E = random(NLINES, Sigma);
Error_rot = zeros(10,100);
Error_trans = zeros(10,100);
i = 1;
while(i~=100)
    E = random(NLINES, Sigma);
    if(E == 0)
        continue;
    end
    Error_rot(:,i) = E(:,1);
    Error_trans(:,i) = E(:,2);
    i = i + 1;
    
end