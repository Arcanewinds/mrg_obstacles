%% 

function dist = point2path(Plan,pathStart)

xp = Plan(:,1);
yp = Plan(:,2);

% 100 is the threshold
d = ones(length(yp)-1,1).* 100;
theta = ones(length(yp)-1,1);

for i = pathStart:length(yp)
    Xp1 = [xp(i) yp(i) 0]';
    Xp0 = [xp(i-1) yp(i-1) 0]';
    
    a = Xp1 - Xr;
    b = Xp1 - Xp0;
    
    b_norm = norm(b);
    a_norm = norm(a);
    
    n = cross(a,b);
    sgn = sign(n(3));
    d(i-1) = sgn * norm(cross(a,b)/b_norm); 
    theta(i-1) = asin(d(i-1)/a_norm);
end

[dist, index] = min(abs(d));