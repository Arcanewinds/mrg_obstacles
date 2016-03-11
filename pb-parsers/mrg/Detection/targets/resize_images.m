function resize_images

train_img = cell(118,1);
img_size = zeros(118,2);
%test_img = cell(30,1);

for i = 1:118
    filename = sprintf('img%d.jpg', i);
    train_img{i} = imread(filename);    
end
%for i = 79:118
%    filename = sprintf('img%d.jpg', i);
%    test_img{i-78} = imread(filename);  
%end

for i = 1:118
    img_size(i,1) = size(train_img{i}, 1); %vert
    img_size(i,2) = size(train_img{i}, 2); %hor
end

[M, I_n] = max(img_size(:,2));
[M, I_f] = min(img_size(:,2));

idx = kmeans(img_size, 3, 'Replicates', 3);
far_idx = idx(I_f);
near_idx = idx(I_n);
if(near_idx + far_idx == 4)
    mid_idx = 2;
elseif(near_idx + far_idx == 5)
    mid_idx = 1;
else
    mid_idx = 3;
end

near_idx = find(idx(:) == near_idx);
mid_idx = find(idx(:) == mid_idx);
far_idx = find(idx(:) == far_idx);

widths = zeros(3,3);
heights = zeros(3,3);

widths(1,1) = min(img_size(far_idx,2));
widths(1,2) = max(img_size(far_idx,2));
widths(1,3) = mean(img_size(far_idx,2));
widths(2,1) = min(img_size(mid_idx,2));
widths(2,2) = max(img_size(mid_idx,2));
widths(2,3) = mean(img_size(mid_idx,2));
widths(3,1) = min(img_size(near_idx,2));
widths(3,2) = max(img_size(near_idx,2));
widths(3,3) = mean(img_size(near_idx,2));
heights(1,1) = min(img_size(far_idx,1));
heights(1,2) = max(img_size(far_idx,1));
heights(1,3) = mean(img_size(far_idx,1));
heights(2,1) = min(img_size(mid_idx,1));
heights(2,2) = max(img_size(mid_idx,1));
heights(2,3) = mean(img_size(mid_idx,1));
heights(3,1) = min(img_size(near_idx,1));
heights(3,2) = max(img_size(near_idx,1));
heights(3,3) = mean(img_size(near_idx,1));
widths(:,3) = round(widths(:,3));
heights(:,3) = round(heights(:,3));
ratios = heights(:,3) ./ widths(:,3);

far_imgs = zeros(heights(1,3), widths(1,3), 3);
mid_imgs = zeros(heights(2,3), widths(2,3), 3);
near_imgs = zeros(heights(3,3), widths(3,3), 3);

for j = 1:length(far_idx)
    i = far_idx(j);
    this_ratio = img_size(i,1) / img_size(i,2);
    if(this_ratio < ratios(1))
        train_img{i} = imresize(train_img{i}, (heights(1,3) / img_size(i, 1)));
        offset = size(train_img{i}, 2) - widths(1,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(:,end, :) = [];
        end
        se = translate(strel(1), [0, -offset/2]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,2) - offset) + 1;
        temp(:,cut:end, :) = [];
        if(size(temp, 1) > heights(1,3))
            temp(end, :, :) = [];
        end   
        if(j == 1)
            far_imgs = temp;
        else
            far_imgs = cat(4, far_imgs, temp);
        end
    else
        train_img{i} = imresize(train_img{i}, (widths(1,3) / img_size(i, 2)));
        offset = size(train_img{i}, 1) - heights(1,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(end,:,:) = [];
        end
        se = translate(strel(1), [-offset/2, 0]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,1) - offset) + 1;
        temp(cut:end,:,:) = [];
        if(size(temp, 2) > widths(1,3))
            temp(:, end, :) = [];
        end   
        if(j == 1)
            far_imgs = temp;
        else
            far_imgs = cat(4, far_imgs, temp);
        end
    end
end
for j = 1:length(mid_idx)
    i = mid_idx(j);
    this_ratio = img_size(i,1) / img_size(i,2);
    if(this_ratio < ratios(2))
        train_img{i} = imresize(train_img{i}, (heights(2,3) / img_size(i, 1)));
        offset = size(train_img{i}, 2) - widths(2,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(:,end, :) = [];
        end
        se = translate(strel(1), [0, -offset/2]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,2) - offset) + 1;
        temp(:,cut:end, :) = [];
        if(size(temp, 1) > heights(2,3))
            temp(end, :, :) = [];
        end    
        if(j == 1) 
            mid_imgs = temp;
        else
            mid_imgs = cat(4, mid_imgs, temp);
        end
    else
        train_img{i} = imresize(train_img{i}, (widths(2,3) / img_size(i, 2)));
        offset = size(train_img{i}, 1) - heights(2,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(end,:,:) = [];
        end
        se = translate(strel(1), [-offset/2, 0]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,1) - offset) + 1;
        temp(cut:end,:,:) = [];
        if(size(temp, 2) > widths(2,3))
            temp(:, end, :) = [];
        end   
        if(j == 1)
            mid_imgs = temp;
        else
            mid_imgs = cat(4, mid_imgs, temp);
        end
    end
end
for j = 1:length(near_idx)
    i = near_idx(j);
    this_ratio = img_size(i,1) / img_size(i,2);
    if(this_ratio < ratios(3))
        train_img{i} = imresize(train_img{i}, (heights(3,3) / img_size(i, 1)));
        offset = size(train_img{i}, 2) - widths(3,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(:,end, :) = [];
        end
        se = translate(strel(1), [0, -offset/2]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,2) - offset) + 1;
        temp(:,cut:end, :) = [];
        if(size(temp, 1) > heights(3,3))
            temp(end, :, :) = [];
        end   
        if(j == 1)
            near_imgs = temp;
        else
            near_imgs = cat(4, near_imgs, temp);
        end
    else
        train_img{i} = imresize(train_img{i}, (widths(3,3) / img_size(i, 2)));
        offset = size(train_img{i}, 1) - heights(3,3);
        odd = mod(offset, 2);
        offset = offset - odd;
        if(odd)
            train_img{i}(end,:,:) = [];
        end
        se = translate(strel(1), [-offset/2, 0]); 
        temp = imdilate(train_img{i},se);
        cut = (size(temp,1) - offset) + 1;
        temp(cut:end,:,:) = [];
        if(size(temp, 2) > widths(3,3))
            temp(:, end, :) = [];
        end   
        if(j == 1)
            near_imgs = temp;
        else
            near_imgs = cat(4, near_imgs, temp);
        end
    end
end

for j = 1:size(near_imgs, 4)
    filename = sprintf('near%0.3d.jpg', j);
    imwrite(near_imgs(:,:,:,j), filename);
end
for j = 1:size(mid_imgs, 4)
    filename = sprintf('mid%0.3d.jpg', j);
    imwrite(mid_imgs(:,:,:,j), filename);
end
for j = 1:size(far_imgs, 4)
    filename = sprintf('far%0.3d.jpg', j);
    imwrite(far_imgs(:,:,:,j), filename);
end

end
