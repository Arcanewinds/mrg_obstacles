function poles = pole_cluster(scan)
    [ranges, angles] = thresh_detect(scan, 1050);
    num_obs = length(ranges);
    if(num_obs > 0)
    %initialise 
    num_c = 1; %number of clusters
    num_mem = 1; %number of members in current cluster
    poles = zeros(1,2); %cluster angle/range
    a_mean = angles(1); %mean angle
    r_mean = ranges(1); %mean range
    save = 0;
    for m = 2:num_obs
       %take difference between current and last detection
       a_diff = abs(angles(m) - angles(m-1));
       r_diff = abs(ranges(m) - ranges(m-1));  
       if (a_diff < 0.1 && r_diff < 0.1)
           %if subsequent detections are close, update mean
           a_mean = (((a_mean * num_mem) + angles(m))/(num_mem+1));
           r_mean = (((r_mean * num_mem) + ranges(m))/(num_mem+1));
           %increment number of members
           num_mem = num_mem + 1;
           save = 0;
       else
           %if current detection is not close to previous
           %save previous mean
           if(num_c == 1)
                poles(num_c, 2) = a_mean;
                poles(num_c, 1) = r_mean;
           else
                poles = cat(1, poles, [r_mean, a_mean]);        
           end
           %initialise new cluster
           num_mem = 1;
           num_c = num_c + 1;
           a_mean = angles(m);
           r_mean = ranges(m);
           save = 1;
       end
    end
    if(save == 0)
        poles(num_c, 2) = a_mean;
        poles(num_c, 1) = r_mean;
    end
    else
        poles = [];
    end
end