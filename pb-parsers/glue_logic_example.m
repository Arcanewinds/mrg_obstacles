% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
addpath('mrg');

% Load camera model
load('BB2-14200208.mat');
    
% Set up MOOS channel names
laser_channel = 'LMS1xx_11360134_laser2d'; % Modify for your robot
stereo_channel = 'BUMBLEBEE2_IMAGES';
pose_channel = 'BB2-14366971_pose_pb'; % Modify for your robot
control_channel  = 'husky_plan';

% Initialise mex-moos and register channels
clear mexmoos
host = '192.168.0.15'; % Modify for your robot i.e .13 for robot3 .12 for robot2 etc
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client);
mexmoos('REGISTER', laser_channel, 0.0);
mexmoos('REGISTER', stereo_channel, 0.0);
mexmoos('REGISTER', pose_channel, 0.0);
pause(1.0); % Give mexmoos a chance to connect (important!)

% Main loop
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, laser_channel, true);
    stereo_images = GetStereoImages(mailbox, stereo_channel, true);
    relative_poses = GetRelativePoses(mailbox, pose_channel);
    
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Display laser scan
    figure(1);
    ShowLaserScan(scan);

    % Display stereo image
    figure(2);
    ShowStereoImage(stereo_images)
    
    % Display undistorted stereo image
    figure(3);
    ShowStereoImage(UndistortStereoImage(stereo_images, camera_model));

    % Display relative poses
    disp(relative_poses);
    
    pause(0.1); % don't overload moos w/commands
end