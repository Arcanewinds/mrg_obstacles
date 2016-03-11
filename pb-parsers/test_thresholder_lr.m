% clear all
close all
clc
load logStereo2.mat

figure(1);
for i = 1:length(stereoData2)
    % i = 125;
    s = stereoData2{i};
    r_img = s.right.rgb;
    l_img = s.left.rgb;
    figure(1);
    [r_x, r_y] = detect_target(r_img);

    figure(2);
    [l_x, l_y] = detect_target(l_img);

    figure(3);
    clf
    hold on
    if ~isempty(r_x)
        [r_r, r_a] = project_geom(r_x, r_y);
        scatter(r_r*cos(r_a),r_r*sin(r_a));
    end
    if ~isempty(l_x)
        [l_r, l_a] = project_geom(l_x, l_y);
        scatter(l_r*cos(l_a),l_r*sin(l_a));
    end
    axis([0 10 -5 5]);
    hold off;
    drawnow;
    pause(0.1);
end
