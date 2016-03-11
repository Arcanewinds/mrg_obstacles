
n = 0;
for j = 1:length(far_idx)
    i = far_idx(j);
    if(size(train_img{i},1) < heights(1,1) || heights(1,1) == 0)
        heights(1,1) = size(train_img{i},1)
    elseif(size(train_img{i},1) > heights(1,2))
        heights(1,2) = size(train_img{i},1)
    end
    heights(1,3) = ((heights(1,3) * n) + size(train_img{i},1)) / (n + 1); 
    
    if(size(train_img{i},2) < widths(1,1) || widths(1,1) == 0)
        widths(1,1) = size(train_img{i},2)
    elseif(size(train_img{i},2) > widths(1,2))
        widths(1,2) = size(train_img{i},2)
    end
    widths(1,3) = ((widths(1,3) * n) + size(train_img{i},2)) / (n + 1);
    n = n+1;
end
for j = 1:length(mid_idx)
    i = mid_idx(j);
    if(size(train_img{i},1) < heights(2,1) || heights(2,1) == 0)
        heights(2,1) = size(train_img{i},1)
    elseif(size(train_img{i},1) > heights(2,2))
        heights(2,2) = size(train_img{i},1)
    end
    heights(2,3) = ((heights(2,3) * n) + size(train_img{i},1)) / (n + 1); 
    
    if(size(train_img{i},2) < widths(2,1) || widths(2,1) == 0)
        widths(2,1) = size(train_img{i},2)
    elseif(size(train_img{i},2) > widths(2,2))
        widths(2,2) = size(train_img{i},2)
    end
    widths(2,3) = ((widths(2,3) * n) + size(train_img{i},2)) / (n + 1);
    n = n+1;
end
for j = 1:length(near_idx)
    i = near_idx(j);
    if(size(train_img{i},1) < heights(3,1) || heights(3,1) == 0)
        heights(3,1) = size(train_img{i},1)
    elseif(size(train_img{i},1) > heights(3,2))
        heights(3,2) = size(train_img{i},1)
    end
    heights(3,3) = ((heights(3,3) * n) + size(train_img{i},1)) / (n + 1); 
    
    if(size(train_img{i},2) < widths(3,1) || widths(3,1) == 0)
        widths(3,1) = size(train_img{i},2)
    elseif(size(train_img{i},2) > widths(3,2))
        widths(3,2) = size(train_img{i},2)
    end
    widths(3,3) = ((widths(3,3) * n) + size(train_img{i},2)) / (n + 1);
    n = n+1;
end

if(j == 1)
    temp = mid_imgs;
    mid_imgs(:,:,1) = imdilate(train_img{i}(:,:,1),se);
    mid_imgs(:,:,2) = imdilate(train_img{i}(:,:,2),se);
    mid_imgs(:,:,3) = imdilate(train_img{i}(:,:,3),se);
else
    temp(:,:,1) = imdilate(train_img{i}(:,:,1),se);
    temp(:,:,2) = imdilate(train_img{i}(:,:,2),se);
    temp(:,:,3) = imdilate(train_img{i}(:,:,3),se);
    mid_imgs = cat(4, mid_imgs, imdilate(train_img{i},se));
end
