function [ xstate ] = slam(polePos, mailbox, channel, last_state)

%%
% xstate = struct
% xstate.vpose = [x, y, theta]'
% xstate.features = [range_i, bearing_i]
% xstate.covariance = (3+N)x(3+N) cov matrix for all features
%%
% init
% x = [0; 0; 0;];
% x is state [posex, posey, poseth, rangei, bearingi]
features = last_state.features';
x = [last_state.vpose; features(:)];
P = last_state.covariance;

visual_odom = GetVisualOdometry(mailbox, channel, false);
if ~isempty(visual_odom)
    % u: x-forward, y-right, theta-clockwise

    u = compose_poses(visual_odom);
%     vis_odom_pose = tcomp(x(1:3), u);
%     u_history = [u_history; u'];

    [xPred, PPred] = SLAMPrediction(u, x, P);
    x = xPred;
    P = PPred;

    % find poles
%     pole_polar = polePos;
%     [5-loop_idx, 0; loop_idx, pi];

    % do data associations
    if(~isempty(polePos))
        z = SLAMDataAssociations(x, polePos);
    else
        z = [];
    end
    [x, P] = SLAMMeasurement(z, x, P);
else
    disp('No Visual Odometry. Cannot SLAM');
end

xstate = struct;
xstate.vpose = x(1:3);
xstate.features = [x(4:2:end),x(5:2:end)];
xstate.covariance = P;

end

