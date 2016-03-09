function status = update_status(old_status,curSlam,x_ellipse)
status = old_status;

switch old_status
    case 1
        if curSlam.vpose(1) > 11
            status = 3;
        elseif ~isempty(x_ellipse)
            status = 2;
        end
    case 2
%             CHECK IF ARRIVED AT TARGET
    case 3
        if norm(curSlam.vpose(1:2) - [-0.5; 0]) < 0.2
            status = 4;
        end
    case 4
        if (norm(curSlam.vpose(1:2) - [-0.5; 0]) < 0.2) ...
                && mod((curSlam.vpose(3) - pi),2*pi) < pi/8
            status = 5;
        end
    case 5
        disp('YEAH BUDDY!!!');
    otherwise
end