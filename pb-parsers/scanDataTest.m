scanDat = load('../../bigData/scanData3.mat');
scanDat = scanDat.scanLog;
addpath('mrg');
close all
figure(1)
for i = 1:length(scanDat)
    scan = scanDat{i, 1};
    if(~isempty(scan))
        poles = pole_cluster(scan);
        ShowLaserScan(scan);  
        if(~isempty(poles))
            viscircles([poles(:,1).*cos(poles(:,2)) poles(:,1).*sin(poles(:,2))], ones(size(poles,1),1)*0.2);
        end
    end
    %ShowLaserScan(scan);
    drawnow
    pause(0.1)
end