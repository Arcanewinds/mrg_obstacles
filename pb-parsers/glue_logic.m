
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
load('BB2-14366960.mat');
    
% Set up MOOS channel names
laser_channel = 'LMS1xx_14320092_laser2d'; % Modify for your robot
stereo_channel = 'BUMBLEBEE2_IMAGES';
pose_channel = 'BB2-14366960_pose_pb';
control_channel  = 'husky_plan';

% Initialise mex-moos and register channels
mexmoos('CLOSE');
clear mexmoos
host = '192.168.0.14'; % Modify for your robot i.e .13 for robot3 .12 for robot2 etc
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client);
mexmoos('REGISTER', laser_channel, 0.1);
mexmoos('REGISTER', stereo_channel, 0.5);
mexmoos('REGISTER', pose_channel, 0.0);
pause(1.0); % Give mexmoos a chance to connect (important!)
scanLog = 0;
scanLog = cell(6000,1);
stereoLog = 0;
stereoLog = cell(2000,1);
odomLog = 0;
odomLog = cell(10000,3);
poseLog = 0;
poseLog = cell(8000,1);
count1 = 0; count2 = 0; count3 = 0; count4 = 0;
key = 0;
    
% Main loop
 h.fig = figure(1);
 %set(h.fig, 'KeyPressFcn', @(x,y)disp(get(h.fig,'CurrentCharacter')));
    
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, laser_channel, true);
    stereo_images = GetStereoImages(mailbox, stereo_channel, true);
    relative_poses = GetRelativePoses(mailbox, pose_channel);
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Display laser scan
    if (~isempty(scan))
        ShowLaserScan(scan);
        %key = get(h.fig,'CurrentKey');
        %if key == 's'
        poles = pole_cluster(scan);
            count1 = count1 + 1;
            if(count1 < 1000)
                scanLog{count1} = scan;
            end
        %end
    end
    % Display stereo image
    h.fig2 = figure(2);
    if (~isempty(stereo_images))
        %ShowStereoImage(stereo_images)
        %key = get(h.fig2,'CurrentKey');
        %if key == 's'
            count2 = count2 + 1;
            if(count2 < 1000)
            stereoLog{count2} = stereo_images;
            end
        %end
    end
    
    % Display undistorted stereo image
    %figure(3);
    %ShowStereoImage(UndistortStereoImage(stereo_images, camera_model));

    % Display relative poses
    %disp(relative_poses);
    if(~isempty(relative_poses))
        max_i = length(relative_poses);
        for i = 1:max_i-1
            count4 = count4 + 1;
            if(count4 < 1000)
                 poseLog{count4,1}  = relative_poses(i);
            end
        end
    end
    
    visual_odom = GetVisualOdometry(mailbox, pose_channel, false);
    if(~isempty(visual_odom))
        max_i = length(visual_odom);
        for i = 1:max_i
            count3 = count3 + 1;
            if(count3 < 1000)
                odomLog{count3,1} = visual_odom{i}.x;
                odomLog{count3,2} = visual_odom{i}.y;
                odomLog{count3,3} = visual_odom{i}.theta;
            end
        end
    end
    
    pause(0.1); % don't overload moos w/commands
end
