function train_hog

for i = 1:58
    filename = sprintf('far%0.3d.jpg', i);
    if(i == 1)
        far_img = imread(filename);
    else
        far_img = cat(4, far_img, imread(filename));    
    end
end
for i = 1:43
    filename = sprintf('mid%0.3d.jpg', i);
    if(i == 1)
        mid_img = imread(filename);
    else
        mid_img = cat(4, mid_img, imread(filename));    
    end
end
for i = 1:17
    filename = sprintf('near%0.3d.jpg', i);
    if(i == 1)
        near_img = imread(filename);
    else
        near_img = cat(4, near_img, imread(filename));    
    end
end

hogCellSize = 4;
farHog = {} ;
for i = 1:size(far_img, 4)
  farHog{i} = vl_hog(far_img(:,:,:,i), hogCellSize) ;
end
farHog = cat(4, farHog{:});
wfar = mean(farHog, 4) ;
save('class/far_hog.mat', 'wfar') ;

midHog = {} ;
for i = 1:size(mid_img, 4)
  midHog{i} = vl_hog(mid_img(:,:,:,i), hogCellSize) ;
end
midHog = cat(4, midHog{:});
wmid = mean(midHog, 4) ;
save('class/mid_hog.mat', 'wmid') ;

nearHog = {} ;
for i = 1:size(near_img, 4)
  nearHog{i} = vl_hog(near_img(:,:,:,i), hogCellSize) ;
end
nearHog = cat(4, nearHog{:});
wnear = mean(nearHog, 4) ;
save('class/near_hog.mat', 'wnear') ;


figure()
subplot(1,3,1);
imagesc(vl_hog('render', wfar)) ;
hold on
colormap gray ;
axis equal ;
title('HOG Near model') ;
hold off
subplot(1,3,2);
imagesc(vl_hog('render', wmid)) ;
hold on
colormap gray ;
axis equal ;
title('HOG Mid model') ;
hold off
subplot(1,3,3);
imagesc(vl_hog('render', wnear)) ;
hold on
colormap gray ;
axis equal ;
title('HOG Far model') ;
hold off


