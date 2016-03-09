%%GLUE
% Create paths to all folders
mexmoos('CLOSE')
clear mexmoos
% Add MRG helper functions
addpath(genpath('mrg'));

% Load camera model
load('BB2-14200208.mat');
    
% Set up MOOS channel names
channels.laser_channel   = 'LMS1xx_14320092_laser2d';
channels.stereo_channel  = 'BUMBLEBEE2_IMAGES';
channels.pose_channel    = 'BB2-14366971_pose_pb'; 
channels.control_channel = 'husky_plan';

% Initialise mex-moos and register channels
clear mexmoos
host = '192.168.0.14'; % Modify for your robot
client = 'Team';
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client);
mexmoos('REGISTER', channels.laser_channel, 0.0);
mexmoos('REGISTER', channels.stereo_channel, 0.0);
mexmoos('REGISTER', channels.pose_channel, 0.0);
pause(0.1); % Give mexmoos a chance to connect (important!)
%% Parameters
velocity = 0.3;
omega = 0.3;
planLength = 3;
obsThresh = 650;

%Initialise
lastSlam.vpos = [0 0 0]';
lastSlam.features = [];
lastSlam.cov = 0.0005 * diag([1,1,0.01]);

while true
    % Subscribe to channels in one place and pass handles
    % laser_channel = 'LMS1xx_14320092_laser2d'; % Modify for your robot
    % stereo_channel = 'BUMBLEBEE2_IMAGES';
    % pose_channel = 'BB2-14366960_pose_pb';
    % control_channel  = 'husky_plan';
    
    
    %% Call Tims stuff
    % Subscribe to laser_channel, stereo_channel
    % Outputs: pole position angle and distance [range angle; range angle]
    %           target [range, angle]
    
    
    %% Call Kevin's stuff
    % Inputs: [range, angle; range angle...]
    %           subscribe channels: pose_channel
    % Outputs: State vector [x y theta range angle range angle ... ]'
    
    %% Call gregs planner
    % Inputs: obstacle [angle, range; ...] Tim ?
    %           pole [angle, range] from Kevin
    %           position in [x;y;theta;range;angle;range;angle...] Kevin
    % Outputs: Global x, y locations for plan [x y; x y; ...]
    
    %% Call Adam's Controller
    % Inputs: Global Robot from Slam [x y theta]'
    %         Global Path from Planner [x,y; x,y ... ]
    % Subscribe to control_channel
    
    % Output: vehicle movement
    
    %% Flag struct
    % flags.needPlan => need to make a new plan
    
    
    mailbox = mexmoos('FETCH');
      
    %% Implement Pole Detection -> SLAM -> Planner (If required) -> Controller
    
    scan           = GetLaserScans(mailbox, channels.laser_channel, true);
    polePos        = pole_cluster(scan);
    [obstacle_ranges, obstacle_angles] = thresh_detect(scan,obsThresh);
    obstacles = [obstacle_ranges obstacle_angles];
    curSlam        = slam(polePos, mailbox, channels.pose_channel, lastSlam);
    
    if flags.needPlan == 1
        Xplan = planner(curSlam,obstacles);
    end
    
    [i, flags] = controller2(channels,Xplan,curSlam,velocity,omega,i,flags);
end
    