scanData = load('./pb-parsers/logScan2.mat');
scanData = scanData.scanData2;
% addpath('mrg');
close all
figure(1)
for i = 1:length(scanData)
    scan = scanData{i, 1};
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