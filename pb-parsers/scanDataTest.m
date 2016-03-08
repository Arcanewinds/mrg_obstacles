scanDat = load('scanData3.mat');
scanDat = scanDat.scanLog;
addpath('mrg');
figure(1)
for i = 1:1000
    scan = scanDat{i, 1};
    subplot(1,2,1);
    ShowLaserScan(scan);
    I_ref = scan.reflectances(:) > 600;
    M_ref = scan.reflectances(I_ref);
    M_range = scan.ranges(I_ref);
    scan.reflectances(:) = 0;
    scan.ranges(:) = 0;
    scan.reflectances(I_ref) = M_ref;
    scan.ranges(I_ref) = M_range;
    subplot(1,2,2);
    ShowLaserScan(scan);
    drawnow
end