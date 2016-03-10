%%GLUE
% Create paths to all folders
mexmoos('CLOSE')
clear mexmoos
close all
% Add MRG helper functions
addpath(genpath('mrg'));

% Load camera model
load('BB2-14200208.mat');
    
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
% mexmoos('REGISTER', channels.stereo_channel, 0.5);
mexmoos('REGISTER', channels.pose_channel, 0.0);
scanLog = 0;
scanLog = cell(6000,1);
stereoLog = 0;
stereoLog = cell(2000,1);
odomLog = 0;
odomLog = cell(10000,3);
poseLog = 0;
poseLog = cell(8000,1);
count1 = 0; count2 = 0; count3 = 0; count4 = 0;
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
%         disp(['Scan received ' num2str(counter)]);
        figure(1)
        suboplot(1,2,1)
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
    [curSlam, new_feature] = slam(polePos, mailbox, channels.pose_channel, lastSlam);
    lastSlam = curSlam;
    
    if(new_feature)
        flags.needPlan = 1;
    end
    
    suboplot(1,2,2)
%     disp([curSlam.vpose(1), curSlam.vpose(2)])
    hold on
    plot(curSlam.vpose(1), curSlam.vpose(2),'rx');
    scatter(curSlam.features(:,1),-curSlam.features(:,2),'bo');
    hold off
%     figure(3)
%     ShowStereoImage(UndistortStereoImage(GetStereoImages(mailbox, stereo_channel, true), camera_model));
    status = update_status(status,curSlam,x_ellipse);
    if status == 5
        break
    end
    
    if flags.needPlan == 1 && ~isempty(lastScan)
        [obstacle_ranges, obstacle_angles] = thresh_detect(lastScan,obsThresh);
        obstacles = [obstacle_ranges obstacle_angles];
        [plan, flags.badStart] = planner(curSlam,obstacles,oldPlan,planStepCount,status,x_ellipse);
        oldPlan = plan;
        planStepCount = 1;
        flags.needPlan = 0;
        figure(2)
        axis equal
%         set(gca,'Ydir','reverse');
        plot(plan(:,1),-plan(:,2),'g-')
        disp('Plan updated');
    end
    
    if ~isempty(plan)
        [planStepCount, flags] = controller2(channels,plan,curSlam,velocity,omega,planStepCount,flags);
    end
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

    % Display relative poses
    %disp(relative_poses);
%     if(~isempty(relative_poses))
%         max_i = length(relative_poses);
%         for i = 1:max_i-1
%             count4 = count4 + 1;
%             if(count4 < 1000)
%                  poseLog{count4,1}  = relative_poses(i);
%             end
%         end
%     end
    
    visual_odom = GetVisualOdometry(mailbox, channels.pose_channel, false);
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
    pause(0.1)
%     disp(toc);
end
    