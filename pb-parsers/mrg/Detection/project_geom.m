function [range, angle] = project_geom(x, y)
    load 'BB2-14366960.mat'
    f = camera_model.right.fx;
    cx = camera_model.right.cx;
    cy = camera_model.right.cy;
    h = 0.82;
    Z = f*h/(y-cy);
    X = (x-cx)*h/y;
    angle = atan(X/Z);
    range = sqrt(X^2 + Z^2);
end