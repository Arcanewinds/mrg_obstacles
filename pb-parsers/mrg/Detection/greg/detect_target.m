function [x, y] = detect_target(img)
x = [];
y = [];

% subplot(1,2,1);
% imshow(img);
img = mean(img,3);
img(img < 200) = 0;
img(1:240,:) = 0;
img(end-20:end,:) = 0;

bw = im2bw(img,0.5);
% subplot(1,2,2);
% imshow(bw);
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