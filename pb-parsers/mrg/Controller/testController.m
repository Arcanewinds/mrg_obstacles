clear all; close all;
% Dummy plan
xp = 0:2:10;
yp = 10*sin(xp);

% Initial robot position
Xr = [0.2 0.6 0]'; %Cartesian
thetar = pi/6;
tdelta = 0.1;
K = 300;
THETA = zeros(K+1,1);
XX  = zeros(K+1,2);
XX(1,:) = Xr(1:2);
THETA(1) = thetar;
figure(1)
plot(xp,yp);

% Initialise path = 2
path = 2;
for k = 1:K
    figure(1)
hold on
scatter(Xr(1),Xr(2))
bearing = [Xr(1),Xr(2); Xr(1) +  0.4 * cos(thetar),Xr(2) + 0.4 * sin(thetar)];
plot(bearing(:,1),bearing(:,2))
drawnow;
pause(0.05)
d = ones(length(yp)-1,1).* 10;
theta = ones(length(yp)-1,1);

for i = path:length(yp)
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

[error, index] = min(abs(d));

thetaError = thetar - atan((yp(index+1)-yp(index))/(xp(index+1)-xp(index)));
sgnTheta = sign(thetaError);
thetaError = mod(thetaError,2*pi);

% Turn and move
if error > 0.1
omega    = 10 * -d(index);
velocity = 0.3;

% Near line but facing the wrong way
elseif thetaError > 0.5
    velocity = 0;
    omega    = 10 * thetaError * -sign(d(index));

% Moving along nearest piece of path
else omega = 0; velocity = 0.3;
end

pathEnd = sqrt((xp(index+1)-xp(index))^2 + (yp(index+1)-yp(index))^2);
PATHEND(k) = pathEnd;
if index > path - 1 || pathEnd < 0.3 
    path = path + 1;
end

%new position
thetaDelta = omega * tdelta;
% Update vehicle position
Xr = Xr + [velocity * cos(thetar+thetaDelta); velocity * sin(thetar+thetaDelta); 0];
% Update vehicle bearing
thetar = thetar + thetaDelta;
thetar = mod(thetar,2*pi);
THETA(k+1) = thetar;
XX(k+1,:) = Xr(1:2);
OMEGA(k) = (omega);
angle(k) = thetar;
angleerror(k) = thetaError;
end

figure(2)
subplot(1,2,1)
plot(1:K,angle)
subplot(1,2,2)
plot(1:K,angleerror)

figure(4)
plot(1:K,PATHEND)

    
    

