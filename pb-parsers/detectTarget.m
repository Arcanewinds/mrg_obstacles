function location = detectTarget(image)
   
    colors = [1, 0, 0;
            0, 1, 0;
            0, 0, 1;
            0.5, 0.5, 0;
            0, 0.5, 0.5;
            0.5, 0, 0.5;
            0.75, 0.75, 0;
            0, 0.75, 0.75;
            0.75, 0, 0.75;
            0.25, 0.25, 0;
            0, 0.25, 0.25;
            0.25, 0, 0.25];

    c_class = cluster_pixels(image);
    num_c = max(c_class(:,3));
    fig = gcf();
    image2 = zeros(size(image,1), size(image,2), 3);
    image_check = image2;
    c_idx = [];
    best = 0;
    location = [0, 0, 0];
    for j = 1:num_c
        idx = find(c_class(:,3) == j);
        if(length(idx) > 5)
            max_x = max(c_class(idx, 2));
            min_x = min(c_class(idx, 2));
            range_x = max_x - min_x;
            if(range_x > 5)
                max_y = max(c_class(idx, 1));
                min_y = min(c_class(idx, 1));
                range_y = max_y - min_y;
                crop = zeros(range_y, range_x, 3);
                crop_row = c_class(idx, 1) - min_y + 1;
                crop_col = c_class(idx, 2) - min_x + 1;
                for i = 1:length(crop_row)
                    crop(crop_row(i), crop_col(i), :) = image(c_class(idx(i), 1),c_class(idx(i), 2), :);
                end
                crop = imresize(crop,4);
                image = imresize(image, 4);
                [new_best, bestidx] = ellipse_hog(crop);
                if(best < new_best)
                   best = new_best;
                   y = floor(bestidx/ range_y) + min_y;
                   x = mod(bestidx, range_y) + min_x;
                   location = [y, x, best];
                end
%                 for m = 1:length(idx)
%                     image_check(c_class(idx(m), 2), c_class(idx(m), 1), :) = image(c_class(idx(m), 2), c_class(idx(m), 1), :);
%                     %image2(c_class(idx(m), 2), c_class(idx(m), 1), :) = colors((mod(j,11)+1), :);
%                 end
            end
            c_idx = cat(1, c_idx, j);
        else
            c_class(idx,:) = [];
        end
        % image2(c_class(idx, 1), c_class(idx, 2), :) = ones(size(idx, 1),size(idx, 2)) * colors(j, :);
    end
    imshow(image2);
    
end