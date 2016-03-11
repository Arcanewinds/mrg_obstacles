function simple_plan = simplifyPlan(plan,x_obstacles,obstacle_radius)
clean_plan = plan(1,:);
for i = 3:size(plan,1)
    good_point = true;
    seg = [clean_plan(end,:) plan(i,:)];
    for j = 1:size(x_obstacles,1)
        intersect_pts = intersectLineCircle(seg, ...
                                      [x_obstacles(j,:) obstacle_radius]);
        if ~isnan(intersect_pts(1)) && in_segment(intersect_pts,seg)
            good_point = false;
            clean_plan = [clean_plan; plan(i-1,:)];
            break;
        end
    end
end
clean_plan = [clean_plan; plan(end,:)];
simple_plan = clean_plan(1,:);
for i=2:size(clean_plan,1)
    seg_length = norm(clean_plan(i,:) - clean_plan(i-1,:));
    if seg_length > 0.5
        n_split = ceil(seg_length/0.5);
        xs = linspace(clean_plan(i-1,1),clean_plan(i,1),n_split);
        ys = linspace(clean_plan(i-1,2),clean_plan(i,2),n_split);
        simple_plan = [simple_plan; xs(2:end)' ys(2:end)'];
    else
        simple_plan = [simple_plan; clean_plan(i,:)];
    end
end

end