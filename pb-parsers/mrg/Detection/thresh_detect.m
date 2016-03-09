function [ranges, angles] = thresh_detect(scan, thresh)

    %trim edge false self-detections
    scan.reflectances(1:65) = 0;
    scan.reflectances(end-65:end) = 0;
    
    %binary map of reflectances > thresh
    I_ref = scan.reflectances(:) > thresh;
    
    %Calculate Radian-angle for each detection
    angle_base = 0:1:length(I_ref)-1;
    angle_true = angle_base(I_ref).';
    angles = -((angle_true(:) -1)*-scan.step_size - scan.start_angle + 90) * pi/180;

    %Grab Range (m) for each detection
    ranges = scan.ranges(I_ref);
end