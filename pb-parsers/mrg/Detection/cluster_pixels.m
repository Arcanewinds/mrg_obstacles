function clusters = cluster_pixels(image)

    [row, col] = find(image(:, :, 1) > 0);
    coords = cat(2, col, row);
    coords(:,3) = find(coords(:,1) > 0);
    clusters = zeros(size(coords,1),3);
    clusters(:,1) = col(:);
    clusters(:,2) = row(:);
   
    num_c = 0;
    cont = 1;
    while(cont)
        unclustered = size(coords,1) - 1; %number to search
        if(unclustered > 5)
            num_c = num_c + 1; %increment number of clusters
            c_members = 1; %Number of members in cluster
            clusters(coords(1,3), 3) = num_c; %add class label to output
            c_idx = coords(1,3);
            del_idx = [];
            coords(1, :) = []; %delete from search list
            THRESHOLD = 2.8; %Euclidean distance, grabs 2 nearest pixels inc. 2x+1y|2y+1x
            id = 0;
            new = 1;
            while(new)
                new = new - 1;
                id = id + 1;
                new_members = 0;
                dist = zeros(unclustered, 1);
                for i = 1:unclustered
                    dist = sqrt(0.8*abs(clusters(c_idx(id), 1) - coords(i, 1)).^2 + 1.25*abs(clusters(c_idx(id), 2) - coords(i, 2)).^2);
                    if(dist < THRESHOLD)
                        clusters(coords(i,3), 3) = num_c;
                        new_members = new_members + 1;
                        del_idx = cat(1,del_idx,i);
                        c_idx = cat(1,c_idx,coords(i,3));
                        new = new+1;
                    end
                end
                unclustered = unclustered - new_members;
                c_members = c_members + new_members;
                if(new_members > 0)
                    st = c_members - new_members;
                    coords(del_idx(st:end), :) = [];
                end
            end    
        else
            cont = 0;
        end
    end
end
            
            
            
            
            
            
            
           