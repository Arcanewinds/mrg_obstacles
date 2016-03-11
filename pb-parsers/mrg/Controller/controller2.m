%% ControllerFunction
% Inputs: channel structure
%         Xplan = planlength x 2
%         curSlam = structure: .vpose = [x y theta]' ,
%                              .features = [range_i, bearing_i]'  N * 2
%                              .covariance = N+3 X N+3
%         velocity = 1 x 1
%         omega    = 1 x 1
%         i = number of nodes into the plan
%         inflags = incoming flag stuct
% Outputs: k = number of nodes into the plan
%          flags structure
%          Robot controls

function [newStepCount, flags] = controller2(channels,Xplan,curSlam,velocity,omega,planStepCount,inflags)
% 2 stage plan
pathDev = 0.2;
angleDev = pi/6;
planLength = 6;
newStepCount = planStepCount;
flags = inflags;

xSlam = curSlam.vpose(1); ySlam = curSlam.vpose(2); thetaSlam = curSlam.vpose(3);

%% ROTATE
% Rotate until Slam angle = path angle
% Work out whether to go in the [-pi/2 pi/2] for positive or the rest of
forward = sign(dot([1 0]',Xplan(planStepCount,:)'-[xSlam ySlam]'));
thetaDesired = atan((Xplan(planStepCount,2)-ySlam)/(Xplan(planStepCount,1)-xSlam));
if forward == -1
    if sign(thetaDesired) == -1
        thetaDesired = thetaDesired + pi;
    else
        thetaDesired = thetaDesired - pi;
    end
    thetaDelta = thetaDesired-thetaSlam;
    if thetaDelta < -5*pi/6
        thetaDelta = thetaDelta + 2*pi;
    end
else
    thetaDelta = thetaDesired-thetaSlam;
end
distDelta  = sqrt((Xplan(planStepCount,1)-xSlam)^2 + (Xplan(planStepCount,2)-ySlam)^2);
% distDelta = norm(Xplan(i,:) - [xSlam ySlam]);
 
if flags.badStart == 1
    SendSpeedCommand(-0.5, 0.5, channels.control_channel);
    flags.needPlan = 1;
elseif flags.badStart == 2
    SendSpeedCommand(-0.5, -0.5, channels.control_channel);
    flags.needPlan = 1;
else
    %% Drive straight
    % Drive until Slam position = path position

    if abs(thetaDelta) > angleDev && distDelta > pathDev
        sgn = sign(thetaDelta);% Negative is anticlockwise (Left)
        omega = min(0.5 * abs(thetaDelta),0.49);
        SendSpeedCommand(0.0, sgn * omega, channels.control_channel);
        %     pause(0.1); % don't overload moos w/commands


        %% Drive straight
        % Drive until Slam position = path position

    elseif distDelta > pathDev
        velocity = min(0.49,distDelta);
        SendSpeedCommand(velocity, 0.0, channels.control_channel)
        %         pause(0.1);
    else
        newStepCount = planStepCount+1;
        SendSpeedCommand(0, 0, channels.control_channel)
        if newStepCount >= planLength || newStepCount > size(Xplan,1)
            flags.needPlan = 1;
        end
    end
end

end