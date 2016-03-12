function status = update_status(old_status,curSlam,x_ellipse)
status = old_status;
global endzone;
switch old_status
    case 1
        if curSlam.vpose(1) > endzone-.5
            status = 3
        elseif ~isempty(x_ellipse)
            status = 2
        end
    case 2
        if norm(curSlam.vpose(1:2) - x_ellipse) < 0.2
            status = 3
        end
    case 3
        if curSlam.vpose(3) > pi/2
            status = 4
        end
    case 4
        if norm(curSlam.vpose(1:2) - [-2; 0]) < 0.2
            status = 5
        end
    case 5
        if (norm(curSlam.vpose(1:2) - [-2; 0]) < 0.2) ...
                && mod((curSlam.vpose(3) + pi),2*pi) < pi/18
            status = 6
        end
    case 6
        disp('YEAH BUDDY!!!');
    otherwise
end