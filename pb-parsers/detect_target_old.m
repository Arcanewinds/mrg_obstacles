function [x, y] = detect_target(img)
x = [];
y = [];

subplot(1,2,1);
imshow(img);
%img = mean(img,3);
%img(img < 200) = 0;
img(1:240,:) = 0;
%img(end-20:end,:) = 0;
%img(1:end/2, :) = 0;


%%% Color Filter
targetDat = load('mrg/Detection/targets/target_stats.mat');
targetDat = targetDat.targetDat;
image_like = zeros(size(img,1), size(img,2));
    for m = 1:size(image,1)
        for n = 1:size(image,2)
             like1 = normpdf(image(m,n,1), targetDat.mean(1), targetDat.variance(1));
             if(like1 > 1)
                like2 = normpdf(image(m,n,2), targetDat.mean(2), targetDat.variance(2));
                like3 = normpdf(image(m,n,3), targetDat.mean(3), targetDat.variance(3));
             else
                 like2 = 0; like3 = 0;
             end
             image_like(m,n) = (like1 + like2 + like3)/3; %%Avg Likelihood for Channels
        end
    end
    [row, col] = find(image_like(:,:) < 1.5); %% THRESHOLD FOR COLOR LIKELIHOOD
    for m = 1:size(row,1)
        img(row(m), col(m), :) = 0;
    end
    bw = im2bw(img, 0.5);
%%%


%bw = im2bw(img,0.5);
subplot(1,2,2);
imshow(bw);
stats = regionprops(bw,'Centroid','Eccentricity','Solidity',...
    'MajorAxisLength','MinorAxisLength','Orientation','Area');
centers = [];
for j=1:length(stats)
    maj = stats(j).MajorAxisLength;
    min = stats(j).MinorAxisLength;
    or = stats(j).Orientation;
    area = stats(j).Area;
    ecc = stats(j).Eccentricity;
    sol = stats(j).Solidity;
    if (abs(or) < 6) && area > 300 && maj/min > 1.3 && maj/min < 5 && sol > 0.9
        center = stats(j).Centroid;
        centers = [centers; center area];
%         hold on;
%         plot(center(1),center(2),'rx','linewidth',3);
%         hold off;
%         subplot(1,2,1);
%         hold on;
%         plot(center(1),center(2),'rx','linewidth',3);
%         hold off;
    end
end
if ~isempty(centers)
    [~, ind] = max(centers(:,3));
    center = centers(ind,:);
    x = center(1);
    y = center(2);
%     hold on;
%     plot(x,y,'bo','linewidth',4);
%     hold off;
%     subplot(1,2,1);
%     hold on;
%     plot(x,y,'rx','linewidth',3);
%     hold off;
end
end