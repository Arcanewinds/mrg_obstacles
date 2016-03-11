%%GLUE
% Create paths to all folders
mexmoos('CLOSE')
clear mexmoos
close all
% Add MRG helper functions
addpath(genpath('mrg'));

% Load camera model
load('BB2-14366960.mat');
    
% Set up MOOS channel names
channels.laser_channel   = 'LMS1xx_14320092_laser2d';
channels.stereo_channel  = 'BUMBLEBEE2_IMAGES';
channels.pose_channel    = 'BB2-14366960_pose_pb'; 
channels.control_channel = 'husky_plan';

% Initialise mex-moos and register channels
clear mexmoos
host = '192.168.0.14'; % Modify for your robot
client = 'Team';
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client);
mexmoos('REGISTER', channels.laser_channel, 0.1);
mexmoos('REGISTER', channels.stereo_channel, 2.0);
mexmoos('REGISTER', channels.pose_channel, 0.0);
% scanLog = 0;
logSize = 10000;
scanLog = cell(logSize,1);
% stereoLog = 0;
stereoLog = cell(logSize,1);
% odomLog = 0;
odomLog = cell(logSize,1);
% poseLog = 0;
poseLog = cell(logSize,1);
planLog = cell(logSize,1);
count1 = 0; count2 = 0; count3 = 0; count4 = 0; count5 = 0;
pause(0.1); % Give mexmoos a chance to connect (important!)
%% Parameters
velocity = 0.3;
omega = 0.3;
planLength = 6;
obsThresh = 700;

%Initialise
lastSlam.vpose = [0 0 0]';
lastSlam.features = [];
lastSlam.covariance = 0.0005 * diag([1,1,0.01]);
lastSlam.timestamp = 0;

lastScan = [];

flags = struct;
flags.needPlan = 1;
plan = [];
oldPlan = [];
polePos = [];

planStepCount = 2;

global endzone;
endzone = 11;

% status:
%   1: explore forward
%   2: go to target
%   3: go to -0.5,0
%   4: park
%   5: terminal
%   0: EMERGENCY, TODO
flags.status = 1;
x_ellipse = [];
counter = 0;
while true
    tic
    mailbox = mexmoos('FETCH');
    
    scan = GetLaserScans(mailbox, channels.laser_channel, true);
    if ~isempty(scan)
        lastScan = scan;
%         disp(['Scan received ' num2str(counter)]);
        figure(1)
        subplot(1,2,1)
        ShowLaserScan(scan);
        polePos = pole_cluster(scan);
        if(~isempty(polePos))
            viscircles([polePos(:,1).*cos(polePos(:,2)) polePos(:,1).*sin(polePos(:,2))], ones(size(polePos,1),1)*0.2);
        end
        drawnow;
%     else
%         pause(0.1);
    end
    
% %     x_ellipse = target_detector(stereo);
% check ellipse position over 10m

    [curSlam, new_feature] = slam(polePos, mailbox, channels.pose_channel, lastSlam);
    lastSlam = curSlam;
    
    if(new_feature)
        flags.needPlan = 1;
    end
    figure(1)
    subplot(1,2,2)
%     disp([curSlam.vpose(1), curSlam.vpose(2)])
    hold on
    plot(curSlam.vpose(1), -curSlam.vpose(2),'rx');
    scatter(curSlam.features(:,1),-curSlam.features(:,2),'wo');
    axis([-10,10,-10,10])
    hold off
%     figure(3)
%     ShowStereoImage(UndistortStereoImage(GetStereoImages(mailbox, stereo_channel, true), camera_model));
    
    old_status = flags.status;
    flags.status = update_status(flags.status,curSlam,x_ellipse);
    if(old_status ~= flags.status)
        planStepCount = 2;
        flags.needPlan = 1;
        oldPlan = [];
    end
    
    if flags.status == 5
        break
    end
    
    stereo_images = GetStereoImages(mailbox, channels.stereo_channel, true);
    if ~isempty(stereo_images)
        [e_range, e_angle] = target_from_stereo(stereo_images,camera_model);
        e_x = curSlam.vpose(1) + e_range*cos(e_angle + curSlam.vpose(3));
        e_y = curSlam.vpose(2) + e_range*sin(e_angle + curSlam.vpose(3));
        if e_x > endzone && e_x < endzone + 3 && abs(e_y) < 5
            x_ellipse = [e_x, e_y];
        end
    end
    
    if flags.needPlan == 1 && ~isempty(lastScan)
        [obstacle_ranges, obstacle_angles] = thresh_detect(lastScan,obsThresh);
        obstacles = [obstacle_ranges obstacle_angles];
        [plan, flags.badStart] = planner2(curSlam,obstacles,oldPlan,planStepCount,flags.status,x_ellipse);
        oldPlan = plan;
        planStepCount = 2;
        flags.needPlan = 0;
        figure(1)
        subplot(1,2,2)
        axis equal
        hold on
        plot(plan(:,1),-plan(:,2),'g-')
        hold off
        disp('Plan updated');
        count5 = count5 + 1;
        lplan = struct;
        lplan.plan = plan;
        lplan.timestamp = lastScan.timestamp;
        planLog{count5} = lplan;
    end
%     a = [flags.needPlan, planStepCount]
    if ~isempty(plan)
        [planStepCount, flags] = controller2(channels,plan,curSlam,velocity,omega,planStepCount,flags);
    end
%     b = [flags.needPlan, planStepCount]
    counter = counter + 1;
    % Display laser scan
    if (~isempty(scan))
%         ShowLaserScan(scan);
        %key = get(h.fig,'CurrentKey');
        %if key == 's'
        poles = pole_cluster(scan);
        count1 = count1 + 1;
        if(count1 < 1000)
            scanLog{count1} = scan;
        end
        %end
    end
   
    
    % Display undistorted stereo image
    %figure(3);
    %ShowStereoImage(UndistortStereoImage(stereo_images, camera_model));
    
    visual_odom = GetVisualOdometry(mailbox, channels.pose_channel, false);
    if(~isempty(visual_odom))
        max_i = length(visual_odom);
        count3 = count3 + 1;
        if(count3 < logSize)
%                 odomLog{count3,1} = visual_odom{i}.x;
%                 odomLog{count3,2} = visual_odom{i}.y;
%                 odomLog{count3,3} = visual_odom{i}.theta;
              odomLog{count3} = visual_odom;
        end
    end
    
    if(~isempty(curSlam))
        count4 = count4 + 1;
        if(count4 < logSize)
              poseLog{count4} = curSlam;
        end
    end
    
    stereo_images = GetStereoImages(mailbox, channels.stereo_channel, true);
    if (~isempty(stereo_images))
        count2 = count2 + 1;
        if(count2 < logSize)
            stereoLog{count2} = stereo_images;
        end
    end
    pause(0.1)
%     disp(toc);
end
    