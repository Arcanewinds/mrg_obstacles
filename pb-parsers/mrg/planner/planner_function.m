function plan = planner_function(curSlam, obstacles_in)

x_vehicle = curSlam.vpos;

% x_vehicle = x(1:3);
% r_features = x(4:2:end);
% theta_features = x(5:2:end);

[n_obstacles, junk] = size(obstacles_in);
theta_obstacles = obstacles_in(:,2) + ones(n_obstacles,1)*x_vehicle(3);
x_obstacles = ones(n_obstacles,1)*x_vehicle(1) + obstacles_in(:,1).*cos(theta_obstacles);
y_obstacles = ones(n_obstacles,1)*x_vehicle(2) + obstacles_in(:,1).*sin(theta_obstacles);
x_obstacles = [x_obstacles y_obstacles];

x_target = [10, 0];

obstacle_radius = 0.5;

figure(1);
scatter(x_obstacles(:,1),x_obstacles(:,2),'rx');
set(gca,'Ydir','reverse');
hold on;
n_rand_points = 1000;
x_points = rand(n_rand_points, 2)*10;
x_points(:,2) = x_points(:,2) - 5;
x_points = [x_vehicle(1:2)'; x_points; x_target];
n_points = n_rand_points + 2;

valid_inds = [];

% check if starting in obstacle
bad_start = 0;
for i=1:n_obstacles
    if norm(x_vehicle(1:2)' - x_obstacles(i,:)) < obstacle_radius
        bad_start = 1;
        disp('DISASTER');
    end
end

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

[n_valid_points, junk] = size(valid_inds);
scatter(x_points(valid_inds,1),x_points(valid_inds,2),'ob');

dt = delaunayTriangulation(x_points);

edge_costs = zeros(n_points);

e = edges(dt);
[n_edges, junk] = size(e);

for i=1:n_edges
    if any(valid_inds == e(i,1)) && any(valid_inds == e(i,2))
        edge_costs(e(i,1),e(i,2)) = norm(x_points(e(i,1),:) - x_points(e(i,2),:));
    end
end
edge_costs = edge_costs + edge_costs';

[cost, route_ids] = dijkstra(edge_costs,1,n_points);

route = x_points(route_ids,:);
viscircles(x_obstacles,ones(n_obstacles,1)*obstacle_radius);
% plot(route(:,1),route(:,2),'k','linewidth',3);
sx = smooth(route(:,1),3);
sy = smooth(route(:,2),3);
plot(sx,sy,'g','linewidth',3);
axis equal
axis tight
drawnow;
hold off
plan = [sx sy];
end