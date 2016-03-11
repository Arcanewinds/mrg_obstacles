function c_class = cluster_pixels(image)
    [row, col] = find(image(:, :, 1) > 0); 
    coords = cat(2, col, row); 
    for i = 1:size(coords,1)
       coords(i, 3) = i;
    end
    num_c = 0;
    c_class = zeros(size(coords,1),3);
    c_class(:,1) = col(:);
    c_class(:,2) = row(:);
    c_scalar = zeros(1,2);
    c_sumDist = zeros(1,2);
    
    iter = 3;
    cont = 1;
    while(cont)
        new_len = size(coords,1);
        if(new_len > 5)
            c_dist = zeros(new_len, 3);
            num_c = num_c + 1; %increment number of clusters
            %cluster(num_c, :) = [mean(coords(:,1)), mean(coords(:,2))]; %calculate new global mean
            cluster(num_c,:) = [coords(1,1), coords(1,2)];
            radius = 10;
            for j = 1:iter %Number of iterations
                for i = 1:new_len %Number of samples
                   c_dist(i, 1) = abs(coords(i, 1) - cluster(num_c, 1));
                   c_dist(i, 2) = abs(coords(i, 2) - cluster(num_c, 2));
                   c_dist(i, 3) = sqrt(c_dist(i, 2).^2 + c_dist(i, 1).^2);
                end
                [M, Ind] = min(c_dist(:, 3)); %Find closest member
                %members = kmeans(c_dist, 2); %Split distances, those that belong and those that don't
                %c_mem = members == members(Ind); %Find group containing closest member
%                 if (j == 1 || length(c_mem) == 1)
%                    thresh = 12;
%                 else
%                    thresh = c_sumDist(num_c,1) / size(c_mem, 1) * c_sumDist(num_c,2) / size(c_mem, 1);
%                    thresh = max_dist * (1 / 1+thresh);
%                 end
                expand = 1;
                while(expand)
                    old_prop = 0;
                    c_mem = find(c_dist(:, 3) < radius);
                    %avg_dist = sum(c_dist(c_mem)) / size(c_mem,1);
                    prop = sum(c_dist(c_mem)) / (pi*radius)^2;
                    if (prop > 0.3 && ((old_prop*1.05) < prop));
                       disp('expanding cluster');
                       radius = radius*1.1;
                       old_prop = prop;
                    else
                       expand = 0;
                    end
                end
                cluster(num_c, :) = [mean(coords(c_mem, 1)), mean(coords(c_mem, 2))]; %Update cluster center & repeat
                %c_scalar = [mean(abs(coords(c_mem, 1).^2)), mean(abs(coords(c_mem, 2)).^2)];
                %c_sumDist(num_c, :) = sum(c_dist(c_mem, 1:2)) + abs(c_scalar(1, :) - cluster(num_c, :).^2);
                %density = [1 1] / (1 + c_sumDist(num_c, :));
                
                i_mem = coords(c_mem, 3);
            end
            c_class(i_mem,3) = num_c; %Assign found members to cluster
            coords(c_mem, :) = []; %Remove them from set
        else
            cont = 0; %If less than 5 samples remain, stop clustering.
        end
    end
end