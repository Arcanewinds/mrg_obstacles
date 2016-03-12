addpath(genpath('./mrg/'));
% addpath('./mrg');
% addpath('../PoleDetector');
addpath(genpath('./ekfslammatlabcdt/'));

scanData = load('./logScan3.mat');
scanData = scanData.scanData3;
poseData = load('./logPose3');
poseData = poseData.poseData3;

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

states = [last_state];
for i=1:10:length(poseData)-10
    j = 1;
    for k=j:length(scanData)
        if scanData{k}.timestamp < poseData{i}.src_timestamp
            break
        end
    end
    j = k;
    poles = pole_cluster(scanData{j,1});
    last_state = slam_nofeed(poles, poses(i:i+10), last_state);
    states = [states, last_state];
end

Xg = zeros(length(states),1);
Yg = zeros(length(states),1);
thetag = zeros(length(states),1);
for i=1:length(states)
    Xg(i) = states(i).vpose(1);
    Yg(i) = states(i).vpose(2);
    thetag(i) = states(i).vpose(3);
end

plot(Yg,Xg)