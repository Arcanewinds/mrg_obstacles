% clear all
close all
clc
load logStereo2.mat

figure(1);
for i = 1:length(stereoData2)
    % i = 125;
    s = stereoData2{i};
    img = s.right.rgb;
    subplot(1,2,1);
    imshow(img);
    title(num2str(i));
    img = mean(img,3);

    img(img < 200) = 0;
    img(1:240,:) = 0;
    img(end-20:end,:) = 0;

    % imshow(img);

    bw = im2bw(img,0.5);
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
            hold on;
            plot(center(1),center(2),'rx','linewidth',3);
            hold off;
            subplot(1,2,1);
            hold on;
            plot(center(1),center(2),'rx','linewidth',3);
            hold off;
        end
    end
    if ~isempty(centers)
        [m, ind] = max(centers(:,3));
        center = centers(ind,:);
        [r, a] = project_geom(center(1),center(2));
        hold on;
        plot(center(1),center(2),'bo','linewidth',4);
        hold off;
        subplot(1,2,1);
        hold on;
        plot(center(1),center(2),'rx','linewidth',3);
        hold off;
    end
    drawnow;
    pause(0.1);
end
