function [ xstate ] = slam_nofeed(polePos, poses, last_state)

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

visual_odom = poses;
% pole_cart = last_state.features; 
% if(~isempty(last_state.features))
%     pole_cart = [pole_cart(4:2:end)', pole_cart(5:2:end)'];
% end
if ~isempty(visual_odom)
    % u: x-forward, y-right, theta-clockwise

    u = compose_poses(visual_odom);
%     vis_odom_pose = tcomp(x(1:3), u);
%     u_history = [u_history; u'];

    [xPred, PPred] = SLAMPrediction(u, x, P);
%      x = [xPred(1:3), pole_cart(:)];
%     x = xPred;
%     P = PPred;
%     poles_pred = [];
%     if(~isempty(x(4:end)))
%         for i=1:length(x(4:end))/2
%             poles_pred = [poles_pred; tcomp(vis_odom_pose, [x((i-1)*2+4:(i-1)*2+5); 0])'];
%         end
%         poles_pred = poles_pred(:,1:2)';
%         x = [xPred(1:3); poles_pred(:)];
%     end
    x = xPred;
    P = PPred;
    % PREDICT FEATURES??
    % find poles
    pole_polar = polePos;
%     if(~isempty(polePos))
%         pole_cart = repmat(x(1:2)',size(polePos,1),1) + [polePos(:,1).*cos(polePos(:,2)+x(3)), polePos(:,1).*sin(polePos(:,2)+x(3))];
%     end
%     if(~isempty(polePos) && ~isempty(poles_pred))
%         figure(2)
%         hold on
%         scatter(pole_cart(:,1),pole_cart(:,2),'r');
%         scatter(poles_pred(1,:),poles_pred(2,:),'b')
%         hold off
% %         tmp = pole_cart';
% %         x = [xPred(1:3); tmp(:)];
%     end
    % do data associations
    if(~isempty(pole_polar))
        z = SLAMDataAssociations(x, pole_polar);
    else
        z = [];
    end

    [x, P, new_feature] = SLAMMeasurement(z, x, P);
else
    disp('No Visual Odometry. Cannot SLAM');
end

xstate = struct;
xstate.vpose = x(1:3);
xstate.features = [x(4:2:end), x(5:2:end)];
xstate.covariance = P;

end

