function cost = planCost(plan)
cost = 0;
for i=1:size(plan,1)-1
    cost = cost + norm(plan(i+1,:) - plan(i,:));
end
end