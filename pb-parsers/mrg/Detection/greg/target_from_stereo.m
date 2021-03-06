function [range, angle] = target_from_stereo(s,camera_model)
    undistort = UndistortStereoImage(s,camera_model);
    r_img = undistort.right.rgb;
    l_img = undistort.left.rgb;
    [r_x, r_y] = detect_target(r_img);
    [l_x, l_y] = detect_target(l_img);
    range = 0; angle = 0;
    if ~isempty(r_x)
        [r_r, r_a] = project_geom(r_x, r_y, camera_model);
    elseif ~isempty(l_x)
        [l_r, l_a] = project_geom(l_x, l_y, camera_model);
%         range = (r_r + l_r)/2;
%         angle = (r_a + l_a)/2;
    end
end