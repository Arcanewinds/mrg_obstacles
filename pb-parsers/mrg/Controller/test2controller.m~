clear all; close all;
% Dummy plan
xp = 0:2:10;
yp = sin(xp);

% Initial robot position
Xr = [1 -0.2 0]'; %Cartesian
thetar = pi/2;
tdelta = 2;

figure(1)
subplot(1,2,1)
plot(xp,yp);
time = 1:30;
thetaDelta = zeros(length(time),1);
for k = time
    figure(1)
    subplot(1,2,1)
    hold on
    scatter(Xr(1),Xr(2))
    
    bearing = [Xr(1),Xr(2); Xr(1) + Xr(1) * 0.1 * cos(thetar),Xr(2) + Xr(2) * 0.1 * sin(thetar)];
    plot(bearing(:,1),bearing(:,2))
    drawnow;
    pause(0.1)
    d = ones(length(yp)-1,1);
    theta = ones(length(yp)-1,1);
    for i = 2:length(yp)
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
    
    omega    = 0.2 * d(index);
    velocity = 0.5;
    
    %new position
    thetaDelta(k) = omega * tdelta;
    % Update vehicle position
    Xr = Xr + [velocity * sin(thetaDelta(k)) * cos(thetar); velocity * sin(thetaDelta(k)) * sin(thetar); 0];
    % Update vehicle bearing
    thetar = thetar + thetaDelta(k);
    subplot(1,2,2)
    scatter(time(k),thetar)
    drawnow;
    pause(0.1);
    
end
% display(omega)





