stereoDat = load('../../bigData/logStereo2.mat');
stereoDat = stereoDat.stereoData2;
addpath('mrg');
load('BB2-14366960.mat');
figure(1)
len = length(stereoDat);
for i = 1:4:len
    %subplot(2,1,1)
    ShowStereoImage(UndistortStereoImage(stereoDat{i,1}, camera_model));
    %subplot(2,1,2)
    %ShowStereoImage(stereoDat{i,1});
    num = ((i-1)/4);
    Filename=strcat('stereoGif/Frame',num2str(num),'.jpg');
    saveas(gca,Filename,'jpg');
    drawnow
end