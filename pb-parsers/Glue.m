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
mexmoos('REGISTER', channels.laser_channel, 0.1);
mexmoos('REGISTER', channels.stereo_channel, 0.5);
mexmoos('REGISTER', channels.pose_channel, 0.0);

pause(0.1); % Give mexmoos a chance to connect (important!)
%% Parameters
velocity = 0.3;
omega = 0.3;
planLength = 3;
obsThresh = 700;

%Initialise
lastSlam.vpose = [0 0 0]';
lastSlam.features = [];
lastSlam.covariance = 0.0005 * diag([1,1,0.01]);

lastScan = [];

flags = struct;
flags.needPlan = 1;
plan = [];
oldPlan = [];
polePos = [];

planStepCount = 1;
% status:
%   1: explore forward
%   2: go to target
%   3: go to -0.5,0
%   4: park
%   5: terminal
%   0: EMERGENCY, TODO
status = 1;
x_ellipse = [];
counter = 0;
while true
    tic
    mailbox = mexmoos('FETCH');
    
    scan = GetLaserScans(mailbox, channels.laser_channel, true);
    if ~isempty(scan)
        lastScan = scan;
        disp(['Scan received ' num2str(counter)]);
        ShowLaserScan(scan);
        drawnow;
        polePos = pole_cluster(scan);
%     else
%         pause(0.1);
    end
    pause(0.1)
%     x_ellipse = target_detector(stereo);
    curSlam = slam(polePos, mailbox, channels.pose_channel, lastSlam);
    lastSlam = curSlam;
    
    status = update_status(status,curSlam,x_ellipse);
    if status == 5
        break
    end
    
    if flags.needPlan == 1 && ~isempty(lastScan)
        [obstacle_ranges, obstacle_angles] = thresh_detect(lastScan,obsThresh);
        obstacles = [obstacle_ranges obstacle_angles];
        plan = planner(curSlam,obstacles,oldPlan,planStepCount,status,x_ellipse);
        oldPlan = plan;
        planStepCount = 1;
        flags.needPlan = 0;
        disp('Plan updated');
    end
    
    if ~isempty(plan)
        [planStepCount, flags] = controller2(channels,plan,curSlam,velocity,omega,planStepCount,flags);
    end
    counter = counter + 1;
    disp(toc);

end
    