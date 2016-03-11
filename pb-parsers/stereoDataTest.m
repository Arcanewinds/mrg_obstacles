stereoDat = load('../../bigData/logStereo1.mat');
scanDat = load('logScan1.mat');

stereoDat = stereoDat.stereoData1;
scanDat = scanDat.scanData1;

addpath('mrg');
addpath('mrg/Detection');


load('BB2-14366960.mat');
targetDat = load('../../targets/target_stats.mat');
targetDat = targetDat.targetDat;


figure(1)
len = length(stereoDat);
for i = 1:1:len
    %subplot(2,1,1)
    image = stereoDat{i,1}.left.rgb(:,:,:);
    image = imresize(image, 0.25);
    image = double(image) ./ 255;
    image_like = zeros(size(image,1), size(image,2));
    for m = 1:size(image,1)
        for n = 1:size(image,2)
             like1 = normpdf(image(m,n,1), targetDat.mean(1), targetDat.variance(1));
             like2 = normpdf(image(m,n,2), targetDat.mean(2), targetDat.variance(2));
             like3 = normpdf(image(m,n,3), targetDat.mean(3), targetDat.variance(3));
             image_like(m,n) = (like1 + like2 + like3)/3;
        end
    end
    idx = find(image_like(:,:) > 1.5);
    w_idx = floor(idx /size(image, 1)) + 1;
    h_idx = mod(idx, size(image, 1));
    image2 = zeros(size(image,1), size(image,2), 3);
    for m = 1:size(w_idx)
        if(h_idx(m) > (size(image, 1) * 0.5))
        image2(h_idx(m), w_idx(m), :) = image(h_idx(m), w_idx(m), :);
        end
    end
%     subplot(3,1,1)
%     imshow(image);
%     subplot(3,1,2)
%     imshow(image2);
%    
    run mrg/Detection/vlfeat/toolbox/vl_setup
    location = [0, 0, 0];
    location = detectTarget(image2);
    if(location(3) > 0)
        disp(location(1))
        disp(location(2))
        disp(location(3))
    end
%     subplot(1,3,3);
%     for j = 1:num_c
%         idx = find(c_class(:,3) == j);
%         for m = 1:length(idx)
%             image2(c_class(idx(m), 2), c_class(idx(m), 1), :) = colors((mod(j,11)+1), :);
%         end
%         % image2(c_class(idx, 1), c_class(idx, 2), :) = ones(size(idx, 1),size(idx, 2)) * colors(j, :);
%     end
  %  imshow(image2)
    
    %[bestEllipses, e] = findellipse(stereoDat{i,1}.left.rgb);
    %normpdf
    %ShowStereoImage(UndistortStereoImage(stereoDat{i,1}, camera_model));
    %subplot(2,1,2)
    %ShowLaserScan(scanDat{i,1});
    %ShowStereoImage(stereoDat{i,1});
    %num = ((i-1)/4);
    Filename=strcat('../../bigData/stereoGif/2Frame',num2str(i),'.jpg');
    saveas(gca,Filename,'jpg');
    drawnow
end