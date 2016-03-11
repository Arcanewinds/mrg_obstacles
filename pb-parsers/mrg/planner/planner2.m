function [plan, badStart] = planner2(curSlam, obstacles_in, old_plan, planStepCount, status, x_ellipse)
obstacle_radius = 0.5;
plotting = false;
% n_rand_points = 1000;
global endzone;
x_vehicle = curSlam.vpose;

switch status
    case 1
        x_target = [endzone 0];
    case 2
        x_target = [x_vehicle(1) + x_ellipse(1)*cos(x_ellipse(2) + x_vehicle(3)), ...
                    x_vehicle(2) + x_ellipse(1)*sin(x_ellipse(2) + x_vehicle(3))];
    case 3
        x_target = x_vehicle(1:2)' - [1 0];
    case 4
        x_target = [-.5 0];
    case 5
        x_target = [-.5 0];
end

% r_features = x(4:2:end);
% theta_features = x(5:2:end);

n_obstacles = size(obstacles_in,1);
theta_obstacles = obstacles_in(:,2) + ones(n_obstacles,1)*x_vehicle(3);
x_obstacles = ones(n_obstacles,1)*x_vehicle(1) + obstacles_in(:,1).*cos(theta_obstacles);
y_obstacles = ones(n_obstacles,1)*x_vehicle(2) + obstacles_in(:,1).*sin(theta_obstacles);
x_obstacles = [x_obstacles y_obstacles];

% If I have an old plan, check if target unchanged and still works.
trying_old_plans = true;
if planStepCount+1 > size(old_plan,1)
    trying_old_plans = false;
end
if ~isempty(old_plan) && trying_old_plans
    old_plan_good = true;
    if old_plan(end,:) ~= x_target
        old_plan_good = false;
    elseif norm(x_vehicle(1:2)' - old_plan(planStepCount,:)) > 0.5
        old_plan_good = false;
    end
    if old_plan_good
        for i=1:n_obstacles
            seg = [x_vehicle(1:2)' old_plan(planStepCount,:)];
            intersect_pts = intersectLineCircle(seg, ...
                                          [x_obstacles(i,:) obstacle_radius]);
            if ~isnan(intersect_pts(1)) && in_segment(intersect_pts,seg)
                old_plan_good = false;
            end
            old_plan_blocked = false;
            for j=planStepCount:size(old_plan,1)
                if norm(old_plan(j,:) - x_obstacles(i,:)) < obstacle_radius
                    old_plan_good = false;
                    old_plan_blocked = true;
                end
            end
            if old_plan_blocked
                disp('plan changed due to blockage');
            end
        end
    end
    
    if old_plan_good
        plan = [x_vehicle(1:2)'; old_plan(planStepCount+1:end,:)];
        if plotting
            figure(10);
            scatter(x_obstacles(:,1),x_obstacles(:,2),'rx');
            set(gca,'Ydir','reverse');
            hold on;
            viscircles(x_obstacles,ones(n_obstacles,1)*obstacle_radius);
            plot(plan(:,1),plan(:,2),'g','linewidth',3);
            axis equal
            axis tight
            drawnow;
            axis([-1 15 -7.5 7.5]);
            hold off
        end
    end
end

% x_points = rand(n_rand_points, 2)*15;
% x_points(:,2) = x_points(:,2) - 7.5;
% x_points(:,1) = x_points(:,1) - 1;

[xs,ys] = meshgrid(linspace(-1,15,30),linspace(-7.5,7.5,30));
x_points(:,1) = xs(:);
x_points(:,2) = ys(:);

for x = linspace(-1,15,45)
    for y = linspace(-7.5,7.5,45)
        for i=1:n_obstacles
            if norm(x_obstacles(i,:) - [x y]) < 1.2
                x_points = [x_points; [x y]];
                break;
            end
        end
    end
end

x_points = [x_vehicle(1:2)'; x_points; x_target];
n_points = size(x_points,1);

% check if starting in obstacle TODO DO SOMETHING IF I AM!!
badStart = 0;
for i=1:n_obstacles
    if norm(x_vehicle(1:2)' - x_obstacles(i,:)) < obstacle_radius
        if(atan((x_obstacles(i,2)-x_vehicle(2))/(x_obstacles(i,1)-x_vehicle(1))) > 0)
            badStart = 1;
        else
            badStart = 2;
        end
        disp('DISASTER');
    end
end

% check if plan complete
if norm(x_vehicle(1:2)' - x_target) < 0.2
    plan = [x_vehicle(1:2)'; x_target];
    return
end

% construct valid indices (should probably be by line rather than vertex
% collision with obstacle
valid_inds = [];
for i=1:n_points
    valid = 1;
    for j=1:n_obstacles
        if norm(x_points(i,:) - x_obstacles(j,:)) < obstacle_radius
            valid = 0;
        end
    end
    if valid == 1
        valid_inds = [valid_inds; i];
    end
end

dt = delaunayTriangulation(x_points);

edge_costs = zeros(n_points);

e = edges(dt);
n_edges = size(e,1);

for i=1:n_edges
    if any(valid_inds == e(i,1)) && any(valid_inds == e(i,2))
        edge_costs(e(i,1),e(i,2)) = norm(x_points(e(i,1),:) - x_points(e(i,2),:));
    end
end

edge_costs = edge_costs + edge_costs';

[cost, route_ids] = dijkstra(edge_costs,1,n_points);

if ~isempty(old_plan) && trying_old_plans
    if old_plan_good
        if planCost(plan) - cost < 1.5
            plan = simplifyPlan(plan,x_obstacles,obstacle_radius);
%             hold on; scatter(plan(:,1),plan(:,2),'kx','linewidth',3); hold off;
            return
        else
            disp('plan changed due to other short path');
        end
    end
end

route = x_points(route_ids,:);
sx = smooth(route(:,1),3);
sy = smooth(route(:,2),3);

if plotting
    figure(10);
    scatter(x_obstacles(:,1),x_obstacles(:,2),'rx');
    set(gca,'Ydir','reverse');
    hold on;
    scatter(x_points(valid_inds,1),x_points(valid_inds,2),'ob');
    viscircles(x_obstacles,ones(n_obstacles,1)*obstacle_radius);
    plot(sx,sy,'g','linewidth',3);
    axis equal
    axis tight
    drawnow;
    hold off
    axis([-1 15 -7.5 7.5]);

end

plan = flip([sx sy],1);
plan = simplifyPlan(plan,x_obstacles,obstacle_radius);
end