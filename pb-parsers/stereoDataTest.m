stereoDat = load('../../bigData/logStereo1.mat');
scanDat = load('logScan1.mat');

stereoDat = stereoDat.stereoData1;
scanDat = scanDat.scanData1;

addpath('mrg');
load('BB2-14366960.mat');
figure(1)
len = length(stereoDat);
for i = 1:1:len
    %subplot(2,1,1)
    ShowStereoImage(UndistortStereoImage(stereoDat{i,1}, camera_model));
    %subplot(2,1,2)
    %ShowLaserScan(scanDat{i,1});
    %ShowStereoImage(stereoDat{i,1});
    %num = ((i-1)/4);
    Filename=strcat('../../bigData/stereoGif/2Frame',num2str(i),'.jpg');
    saveas(gca,Filename,'jpg');
    drawnow
end