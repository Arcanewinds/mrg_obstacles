function log_visual_odom
close all;
clear;

addpath(genpath('./pb-parsers/'));
addpath('./pb-parsers/mrg');
% addpath('../PoleDetector');
addpath(genpath('./ekfslammatlabcdt/'));


% Set up MOOS channel names
pose_channel = 'BB2-14366960_pose_pb';

% Initialise mex-moos and register channels
clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.14','MOOSNAME','ExampleName');
mexmoos('REGISTER', pose_channel, 0.0);

mailbox = mexmoos('FETCH');

% pause
visual_odom = GetVisualOdometry(mailbox, pose_channel, true);
if ~isempty(visual_odom)
     previous_x = visual_odom.x;
     previous_y = visual_odom.y;
     previous_theta = visual_odom.theta;
else
    pause(1)
    mailbox = mexmoos('FETCH');
    pause(0.2)
    visual_odom = GetVisualOdometry(mailbox, pose_channel, true);
    if ~isempty(visual_odom)
         previous_x = visual_odom{1}.x;
         previous_y = visual_odom{1}.y;
         previous_theta = visual_odom{1}.theta;
    else
        disp('No visual odometry')
        return
    end
end
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    visual_odom = GetVisualOdometry(mailbox, pose_channel, true);
    if(~isempty(visual_odom))
        previous_x = visual_odom{1}.x;
        previous_y = visual_odom{1}.y;
        previous_theta = visual_odom{1}.theta;
    end
end


