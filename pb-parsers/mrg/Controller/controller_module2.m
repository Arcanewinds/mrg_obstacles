%% Contol %%
%% USING ODOMETRY
% Given a vector containing the future postions [ x y ] relative to
% the current pose the controller will implement these positions until the
% next update is available
%   Precondition: run all processes via Mission Control and press Y-A on
%   the remote control

Xpath =    [0.5 0.5;...
            0.5 1;...
            1   1;...
            2   2];

% Add MRG helper functions
addpath('../mrg');
    
% Set up MOOS channel names
control_channel = 'husky_plan';

% Initialise mex-moos and register subscribers
clear mexmoos;
host = '192.168.0.14';
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(1.0); % give mexmoos a chance to connect (important!)

% First tell it not to move at all
SendSpeedCommand(0, 0, control_channel)

% Initialise position at 0,0
% X = zeros(size(x) + [1 0]);
% X(2:end,:) = x;
numCommands = length(X(:,1));

velocity = 0.3;
omega = 0.3;
for i = 2:numCommands
    disp('Send command');
    mailbox = mexmoos('FETCH');
    
    
 angle = atan((Xplan(i,2)-ySlam)/(Xplan(i,1)-xSlam));
    
    % Rotate until Slam angle = path angle
    thetaDelata = 10;
    thetaDesired = atan((Xplan(i,2)-ySlam)/(Xplan(i,1)-xSlam));
    while thetaDelata < pi/8
        
        %% PULL THETASLAM FROM SLAM
        
        thetaDelta = thetaDesired - thetaSlam;
        sgn = -sign(thetaDelta);% Negative is anticlockwise (Left)
        SendSpeedCommand(0.0, sgn * omega, control_channel);
        pause(0.1); % don't overload moos w/commands
        thetaDelta = abs(thetaDesired-thetaSlam);
    end
    
    
    % Drive until Slam position = path position
    distDelta = 10;
    while distDelta < 0.2
        SendSpeedCommand(velocity, 0.0, control_channel);
        %% PULL XSLAM YSLAM FROM SLAM
        
        distDelta  = sqrt((X(i,1)-xSlam)^2 + (X(i,2)-ySlam)^2);
        pause(0.1);
    end
    
end

SendSpeedCommand(0, 0, control_channel)

disp('Plan executed')
