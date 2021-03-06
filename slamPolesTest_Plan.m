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
old_plan = [];
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
    
    scan = scanData{ceil(i/(length(poseData)/length(scanData))),1};
    [obstacle_ranges, obstacle_angles] = thresh_detect(scan,600);
    obstacles = [obstacle_ranges obstacle_angles];
    plan = planner2(last_state, obstacles, old_plan, 1, 1, []);
    
    n_obstacles = size(obstacles,1);
    theta_obstacles = obstacles(:,2) + ones(n_obstacles,1)*x_vehicle(3);
    x_obstacles = ones(n_obstacles,1)*x_vehicle(1) + obstacles(:,1).*cos(theta_obstacles);
    y_obstacles = ones(n_obstacles,1)*x_vehicle(2) + obstacles(:,1).*sin(theta_obstacles);
    x_obstacles = [x_obstacles y_obstacles];
    
    figure(10);
    clf;
    hold on;
    scatter(x_obstacles(:,1),x_obstacles(:,2),'ro');
    scatter(last_state.vpose(1),last_state.vpose(2),'k+','linewidth',3);
    scatter(plan(:,1),plan(:,2),'gx','linewidth',3);
    axis equal
    axis tight
    hold off;
    axis([-1 15 -7.5 7.5]);
    drawnow;

    old_plan = plan;
    
    
    poles = pole_cluster(scan);
    if(~isempty(poles) && max(poles(:,1)) > 10)
        poles(poles(:,1) > 10,:) = [];
    end
%     if(~isempty(poles))
%         viscircles([poles(:,1).*cos(poles(:,2)) poles(:,1).*sin(poles(:,2))], ones(size(poles,1),1)*0.2);
%     end
    
    last_state = slam_nofeed(poles, poses(i:i+step), last_state);
    states = [states, last_state];

    se2 = BuildSE2Transform(last_state.vpose); 
%     figure(1);
%     subplot(1,2,2)
%     clf;
%     axis(axes)
%     axis equal
%     whitebg(gcf,'black');

%     hold on
    trans_tri = se2*triangle;
%     fill(trans_tri(1,:), -trans_tri(2,:), 'w');
    
%     viscircles([obstacles(:,1) obstacles(:,2)], ones(size(obstacles,1),1)*0.5);
%     plot(plan(:,1),plan(:,2),'g','linewidth',3);
    
%     plot(last_state.vpose(1),last_state.vpose(2),'bx');
%     scatter(last_state.features(:,1), -last_state.features(:,2),'g.');
%     if(~isempty(poles))
%         scatter(poles(:,1).*sin(poles(:,2)), poles(:,1).*cos(poles(:,2)),'go');
%     end
%     drawnow
%     figure(10);
%     hold on
%     scatter(last_state.features(:,1), last_state.features(:,2),'ko','linewidth',3);
%     hold off;
%     drawnow();
%     axis([-1 15 -7.5 7.5]);

%     pause(.3)
end
%      scatter(last_state.features(:,1), -last_state.features(:,2),'ro');

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


