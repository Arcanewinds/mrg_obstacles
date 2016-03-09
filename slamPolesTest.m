addpath(genpath('./pb-parsers/'));
addpath('./pb-parsers/mrg');
% addpath('../PoleDetector');
addpath(genpath('./ekfslammatlabcdt/'));
close all

scan_id = 3;

switch scan_id
    case 1
        scanData = load('./pb-parsers/logScan1.mat');
        scanData = scanData.scanData1;
        poseData = load('./pb-parsers/logPose1');
        poseData = poseData.poseData1;
        axes = [-2,10,-3,3];
    case 2
        scanData = load('./pb-parsers/logScan2.mat');
        scanData = scanData.scanData2;
        poseData = load('./pb-parsers/logPose2');
        poseData = poseData.poseData2;
        axes = [-4,10,-8,2];
    case 3
        scanData = load('./pb-parsers/logScan3.mat');
        scanData = scanData.scanData3;
        poseData = load('./pb-parsers/logPose3');
        poseData = poseData.poseData3;
        axes = [-3,7,-3,2];
    otherwise
        return
end
        
step = 5;

last_state = struct;
last_state.vpose = [0;0;0];
last_state.covariance = .0005 * diag([1,1,.01]);
last_state.features = [];

poses = {};
pose_trans = {};
for i = 1:max(size(poseData))
    xyzrpy = poseData{i}.xyzrpy;
    poses{i}.timestamp = poseData{i}.src_timestamp;
    poses{i}.x = xyzrpy(1);
    poses{i}.y = xyzrpy(2);
    poses{i}.theta = xyzrpy(6);
end
for i=1:length(poseData)
    pose_trans{i} = [poses{i}.x, poses{i}.y, poses{i}.theta]';
end
triangle = [1,0,1; -.25,.25,1; -.25,-.25,1]';
states = [last_state];
for i=1:step:length(poseData)-step
%     j = 1;
%     for k=j:length(scanData)
%         scanData{k}.timestamp
%         poseData{i}.src_timestamp
%         if scanData{k}.timestamp > poseData{i}.src_timestamp
%             break
%         end
%     end
%     j = k    
    figure(1)
    subplot(1,2,1)
    scan = scanData{ceil(i/(length(poseData)/length(scanData))),1};
    poles = pole_cluster(scan);
    if(~isempty(poles) && max(poles(:,1)) > 10)
        poles(poles(:,1) > 10,:) = [];
    end
    ShowLaserScan(scan);
    if(~isempty(poles))
        viscircles([poles(:,1).*cos(poles(:,2)) poles(:,1).*sin(poles(:,2))], ones(size(poles,1),1)*0.2);
    end
%     poles = pole_cluster(scan,1});
    last_state = slam_nofeed(poles, poses(i:i+step), last_state);
    states = [states, last_state];
%     last_state.vpose(3)
    se2 = BuildSE2Transform(last_state.vpose); 
    figure(1)
    subplot(1,2,2)
    axis(axes)
    axis equal
    whitebg(gcf,'black');

    hold on
    trans_tri = se2*triangle;
    fill(trans_tri(1,:), -trans_tri(2,:), 'w');
%     plot(last_state.vpose(1),last_state.vpose(2),'bx');
%     scatter(last_state.features(:,1), -last_state.features(:,2),'g.');
%     if(~isempty(poles))
%         scatter(poles(:,1).*sin(poles(:,2)), poles(:,1).*cos(poles(:,2)),'go');
%     end
    drawnow
    pause(.1)
end
    scatter(last_state.features(:,1), -last_state.features(:,2),'ro');

% Xg = zeros(length(states),1);
% Yg = zeros(length(states),1);
% thetag = zeros(length(states),1);
% for i=1:length(states)
%     Xg(i) = states(i).vpose(1);
%     Yg(i) = states(i).vpose(2);
%     thetag(i) = states(i).vpose(3);
% end
size(states(end).features)
% hold on
% plot(Yg,Xg,'b');
% scatter(states(end).features(1:2:end), states(end).features(2:2:end));


