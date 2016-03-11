function [range, angle] = target_from_stereo(s)
    r_img = s.right.rgb;
    l_img = s.left.rgb;
    [r_x, r_y] = detect_target(r_img);
    [l_x, l_y] = detect_target(l_img);
    range = 0; angle = 0;
    if ~isempty(r_x) && ~isempty(l_x)
        [r_r, r_a] = project_geom(r_x, r_y);
        [l_r, l_a] = project_geom(l_x, l_y);
        range = r_r + l_r;
        angle = r_a + l_a;
    end
end