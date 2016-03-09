function plan = planner(curSlam, obstacles_in, old_plan, status, x_ellipse)

obstacle_radius = 0.6;
plotting = false;
n_rand_points = 1000;

x_vehicle = curSlam.vpos;

switch status
    case 1
        x_target = [12 0];
    case 2
        x_target = [x_vehicle(1) + x_ellipse(1)*cos(x_ellipse(2) + x_vehicle(3)), ...
                    x_vehicle(2) + x_ellipse(1)*sin(x_ellipse(2) + x_vehicle(3))];
    case 3
        x_target = [-.5 0];
    case 4
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
if ~isempty(old_plan)
    old_plan_good = true;
    if old_plan(end,:) ~= x_target
        old_plan_good = false;
    end
    for i=1:size(old_plan,1)
        for j=1:n_obstacles
            if norm(old_plan(i,:) - x_obstacles(j,:)) < obstacle_radius
                old_plan_good = false;
            end
        end
    end
    if old_plan_good
        return
    end
end

x_points = rand(n_rand_points, 2)*15;
x_points(:,2) = x_points(:,2) - 7.5;
x_points(:,1) = x_points(:,1) - 1;
x_points = [x_vehicle(1:2)'; x_points; x_target];
n_points = n_rand_points + 2;

% check if starting in obstacle TODO DO SOMETHING IF I AM!!
bad_start = 0;
for i=1:n_obstacles
    if norm(x_vehicle(1:2)' - x_obstacles(i,:)) < obstacle_radius
        bad_start = 1;
        disp('DISASTER');
    end
end

% check if plan complete
if norm(x_vehicle(1:2)' - x_target) < 0.2
    plan = x_target;
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

route = x_points(route_ids,:);
% plot(route(:,1),route(:,2),'k','linewidth',3);
sx = smooth(route(:,1),3);
sy = smooth(route(:,2),3);

if plotting
    figure(1);
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
end

plan = flip([sx sy],1);
end