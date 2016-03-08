function SLAM_module
close all;
clear;

addpath(genpath('./pb-parsers/'));
addpath('./pb-parsers/mrg');
% addpath('../PoleDetector');
addpath(genpath('./ekfslammatlabcdt/'));


% Set up MOOS channel names
laser_channel = 'LMS1xx_14320092_laser2d';
stereo_channel = 'BUMBLEBEE2_IMAGES';
pose_channel = 'BB2-14366960_pose_pb';
% control_channel  = 'husky_plan';

% Initialise mex-moos and register channels
clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.14','MOOSNAME','ExampleName');
mexmoos('REGISTER', laser_channel, 0.0);
mexmoos('REGISTER', stereo_channel, 0.0);
mexmoos('REGISTER', pose_channel, 0.0);


% init
% x = [0; 0; 0;];
x = zeros(13,1);
P = 0.0005 * diag([1,1,0.01]);

% Visualisation has x right, y up
vis_transform = [0 1 0;
                1 0 0;
                0 0 -1];

fig2 = figure;
fig3 = figure;
fig4 = figure;

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

u_history = [];

loop_idx = 1;
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, laser_channel, true);
    visual_odom = GetVisualOdometry(mailbox, pose_channel, true);

    if ~isempty(scan) && ~isempty(visual_odom)
        disp(['Start loop ', num2str(loop_idx)]);
        % u: x-forward, y-right, theta-clockwise

        u = [visual_odom{1}.x-previous_x; visual_odom{1}.y-previous_y;...
            visual_odom{1}.theta-previous_theta];
        
        
        
        % u_history = [u_history; u'];

        previous_x = visual_odom{1}.x;
        previous_y = visual_odom{1}.y;
        previous_theta = visual_odom{1}.theta;


        [xPred, PPred] = SLAMPrediction(u, x, P);
        x = xPred;
        P = PPred;

        % find poles
        set(0, 'currentfigure', fig2);
        clf;
        hold on;
        ShowLaserScan(scan);
        pole_polar = zeros(10,3);
%         [pole_cartesian, pole_polar] = PoleDetector(scan, 1000, 0.1);
%         scatter(pole_cartesian(:,1), pole_cartesian(:,2), 20, 'o', 'red') %, 'filled');

        % do data associations
        z = SLAMDataAssociations(x, pole_polar);

        [x P] = SLAMMeasurement(z, x, P);

%         set(0, 'currentfigure', fig3);
%         plot(u_history);
%         legend('x','y','t');

%         set(0, 'currentfigure',fig4);
%         a = axis;
%         clf;
%         axis(a);hold on;
%         n  = length(x);
%         nF = (n-3)/2;
% 
%         x_vis = vis_transform * x(1:3);
%         P_vis = P(1:3,1:3)';
%         DoVehicleGraphics(x_vis,P_vis,3,[0 1]);
% 
%         for i=1:size(z,1)
%             if z(i, 3) > 0
%                 iF = 3 + 2*z(i,3) -1;
%                 xFeature = [x(iF), x(iF+1)];
%                 xFeature = vis_transform(1:2,1:2) * xFeature';
% 
%                 h = line([x_vis(1),xFeature(1)],[x_vis(2),xFeature(2)]);
%                 set(h,'linestyle',':');
%             end
%         end
% 
% %         for i=4:2:size(x,1)
% %             xFeature = [x(i), x(i+1)];
% %             xFeature = vis_transform(1:2,1:2) * xFeature';
% %
% %             h = line([x_vis(1),xFeature(1)],[x_vis(2),xFeature(2)]);
% %             set(h,'linestyle',':');
% %         end
% 
%         for i = 1:nF
%             iF = 3+2*i-1;
%             xFeature = [x(iF), x(iF+1)];
%             xFeature = vis_transform(1:2,1:2) * xFeature';
% 
%             PFeature = P(iF:iF+1, iF:iF+1)';
%             plot(xFeature(1),xFeature(2),'b*');
%             PlotEllipse(xFeature,PFeature,3);
%         end;
% 
%         xlim([-4 4]);
%         ylim([-4 4]);
%         drawnow;
% 
%         loop_idx = loop_idx + 1;
    end % end scan not empty
    x

end

%-------- Drawing Covariance -----%
function eH = PlotEllipse(x,P,nSigma)
eH = [];
P = P(1:2,1:2); % only plot x-y part
x = x(1:2);
if(~any(diag(P)==0))
    [V,D] = eig(P);
    y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
    el = V*sqrtm(D)*y;
    el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
    eH = line(el(1,:),el(2,:) );
end;
