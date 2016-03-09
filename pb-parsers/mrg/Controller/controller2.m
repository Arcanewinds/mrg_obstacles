%% ControllerFunction
% Inputs: channel structure
%         Xplan = planlength x 2
%         curSlam = structure: .vpos = [x y theta]' ,
%                              .features = [range_i, bearing_i]'  N * 2
%                              .covariance = N+3 X N+3
%         velocity = 1 x 1
%         omega    = 1 x 1
%         i = number of nodes into the plan
%         inflags = incoming flag stuct
% Outputs: k = number of nodes into the plan
%          flags structure
%          Robot controls

function [k, flags] = controller2(channels,Xplan,curSlam,velocity,omega,i,inflags)
planLength = 3;
flags = inflags;

xSlam = curSlam.vpos(1); ySlam = curSlam.vpos(2); thetaSlam = curSlam.vpos(3);

%% ROTATE
% Rotate until Slam angle = path angle
thetaDesired = atan((Xplan(i,2)-ySlam)/(Xplan(i,1)-xSlam));
thetaDelta = thetaDesired-thetaSlam;
distDelta  = sqrt((Xplan(i,1)-xSlam)^2 + (Xplan(i,2)-ySlam)^2);
% distDelta = norm(Xplan(i,:) - [xSlam ySlam]);


if abs(thetaDelta) > pi/8
    sgn = -sign(thetaDelta);% Negative is anticlockwise (Left)
    SendSpeedCommand(0.0, sgn * omega, control_channel);
    pause(0.1); % don't overload moos w/commands
    
    
    %% Drive straight
    % Drive until Slam position = path position
    
elseif distDelta > 0.2
        SendSpeedCommand(velocity, 0.0, channels.control_channel)
        pause(0.1);
else
    k = i+1;
    SendSpeedCommand(0, 0, channels.control_channel)
    if k >= planLength
        flags.needPlan = 1;
    end
end

end