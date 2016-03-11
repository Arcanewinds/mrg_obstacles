function [best, bestIndex] = ellipse_hog(image)

addpath('mrg/Detection/matconvnet');
addpath('mrg/Detection/vlfeat');
addpath('mrg/Detection/vlfeat/toolbox');

setup

filename = sprintf('near_hog.mat');
near_hog = load(filename);   
near_hog = near_hog.wnear;
filename = sprintf('mid_hog.mat');
mid_hog = load(filename);    
mid_hog = mid_hog.wmid;
filename = sprintf('far_hog.mat');
far_hog = load(filename);
far_hog = far_hog.wfar;

height = size(image,1);
width = size(image,2);
if(width > 57 && height > 28)
    dhog = near_hog;
elseif(width > 39 && height > 14)
    dhog = mid_hog;
elseif(width > 21 && height > 28)
    dhog = far_hog;
else
    w = 21/width;
    h = 7/height;
    if(w>h)
        image = imresize(image,w); 
    else
        image = imresize(image,h); 
    end
    dhog = far_hog;
end

hogCellSize = 1;
im = im2single(image(:,:,:)) ;
hog = vl_hog(im, hogCellSize) ;
scores = vl_nnconv(hog, dhog, []) ;

    figure(3) ; clf ;
    subplot(2,1,1);
    hold on
    imagesc(scores) ;
    title('Detection') ;
    colorbar ;
    hold off
    [best, bestIndex] = max(scores(:)) ;
    
%     x = floor((bestIndex / size(scores, 1)) * ((width/2) / size(scores, 2)));
%     y = round(mod(bestIndex, size(scores, 1)) * ((height/2) / size(scores, 1)));
%     %subplot(2,1,2);
%     for m = x-5:x+5
%         for n = y-5:y+5
%            if(n > 0 && m > 0)
%                image(n,m,1,i) = 255;
%                image(n,m,2,i) = 0;
%                image(n,m,3,i) = 0;
%            end
%         end
%     end
%     for m = x-6:x+6
%        if(y-6 > 0 && m > 0)
%         image(y-6,m,:,i) = [255, 255, 255];
%        end
%     end
%     for m = x-6:x+6
%        if(y+6 < size(image,1) && m > 0)
%         image(y+6,m,:,i) = [255, 255, 255];
%        end
%     end
%     for n = y-6:y+6
%        if(x-6 > 0 && n > 0)
%         image(n,x-6,:,i) = [255, 255, 255];
%        end
%     end
%     for n = y-6:y+6
%          if(x+6 < size(image,2) && n > 0)
%             image(n,x+6,:,i) = [255, 255, 255];
%          end
%     end
    subplot(2,1,2);
    imshow(image, 'Border', 'tight');
    drawnow

end


