load logStereo2.mat
all_data = [];
for i = 1:length(stereoData2)
    s = stereoData2{i};
    img = s.right.rgb;
    img = mean(img,3);
    all_data = [all_data; img(:)];
end
histogram(all_data);