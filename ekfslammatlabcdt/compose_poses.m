function [ u ] = compose_poses( poses )
%     transform = tcomp([poses{end - 1}.x, poses{end - 1}.y, poses{end - 1}.theta],...
%         [poses{end}.x, poses{end}.y, poses{end}.theta]);
    transform = tcomp([poses{1}.x, poses{1}.y, poses{1}.theta]',...
        [poses{2}.x, poses{2}.y, poses{2}.theta]');
%     for i = len(poses)-1:-1:1
    for i = 2:length(poses)
%         transform = tcomp([poses{i}.x, poses{i}.y, poses{i}.theta], transform);
        transform = tcomp(transform, [poses{i}.x, poses{i}.y, poses{i}.theta]');
    end
    u = transform;
%     u = tinv(transform);
end