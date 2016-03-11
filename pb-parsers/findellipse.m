function  [bestEllipses, e] = findellipse(image)
%
% NOTE: Last tested on Matlab 7.3.0 (R2006b)
%
% Uses the a Hough transform to find ellipses in an image. Does not 
% save the orientation of the ellipse.
%
% Requires the Image and Symbolic toolboxes
%
% Parameter:
%
%     image - the image to find ellipses in. No preprocessing is done
%       on the image. Works best if the image is an edge image.
%
%     All other parameters are loaded from findellipseParams.m edit
%       that file directly to change them.
%
%  Return:
%       bestEllipses - The ellipses found in a matrix of the form:
%       
%           x-center |  y-center | semiMajorAxis | semiMinorAxis | score 
%
%       e - the image after the ellipses found are removed from it. 
%
%
% Author:   Samuel Inverso
% File:     $Id: findellipse.m,v 1.11 2004/10/22 21:25:31 sai6189 Exp sai6189 $
%
% Revisions:
%   2007 07 31 - sai - Now using isempty to check if the similar vector
%                   has elements. This seems to fix the Matrix dimensions must
%                   agree error on line 442
%   2004 10 22 - sai - Original Distributed Version
%
% Note: every where it says majorAxis and minorAxis it should
%   be semiMajorAxis and semiMinorAxis.  
%
% License:
% Use this code freely, it is distributed without warranty express or 
% implied.


% Note to self: REMEMBER point is done in rows and columns 
%   wich means y then x !!!!

% matlab will use the same random numbers on startup if I don't do this
rand('state',sum(100*clock));
%rand('state',0);
%rand('state', [ 0.0856 0.3110 0.6025 0.8911 0.5335 0.0176 0.1073 0.3154 0.0996 0.4711 0.9508 0.7800 0.2692 0.7246 0.2369 0.6645 0.1847 0.0987 0.3092 0.5412 0.1697 0.7410 0.3090 0.4577 0.4978 0.2628 0.5641 0.5789 0.3223 0.7714 0.4618 0.6728 0 0.0000 0.0000 ]' );

%
% Load all of the ellipse detection algorithm parameters 
%
findellipseParams


% make sure the accumulator, bestEllipses, and all symbols are clear
% then remake them.
 
clear  bestEllipses accumulator x y m a b c; 
syms x y m a b c; 
% accumulator is an matrix with columns: xCent  yCent majorAxis minorAxis score
accumulator = [];
bestEllipses = [];

lineeq = m*x + b - y; % equation of a line 
ellipseFcn = a*(x^2) + 2*b*x*y + c*(y^2) - 1; % equation for an ellipse


warning off;

nc = [ -1, 1, 1, -1 ];
nr = [-1 -1, 1, 1 ];
nc = nc * 3;
nr = nr *3;


% this used to be a script where the image was loaded and then
% an edge image was created from the original and set to variable 'e'
% 'e' is then used throughout the code. So instead of looking through
% 500 lines of code to change e to image I just set it here.
e = image;

% Need the size alot so get it, save it, and reduce computation time.
esize = size(e);

% find the indicies in e for non-zero points
%   first find the indicies of the non zero points
%   then randomly select groups of three from those
%   points based on the index into nonZinds
%   then convert the nonZinds index into the real indicies
%   in e
nonZinds  = find(e); 

epoch = 0;
epochsWithoutFindingEllipse = 0;
while  epoch < maxEpochs && ...
    epochsWithoutFindingEllipse < maxEpochsWithoutFindingEllipse  
epoch = epoch + 1;
epochsWithoutFindingEllipse = epochsWithoutFindingEllipse + 1;

fprintf( 'epoch %g\n', epoch); 

% 
% Generate a sets of 3 random indexes into the image matrix.
% First we generate a bunch of random numbers
% Then we make sure those numbers are unique
% then we group them into sets of three
% This gurentees us that we are getting some unique points
% I say some unique points because if we loose points by removing
% the non-unique ones we have add enough to get us up to 3*iterations
% so we may add non-unique points. This is a speed, complexity, how much
% does it really matter technicallity.
%
randIndexes = rand(1,iterations*3);
randIndexes = unique(randIndexes );
numIndexGroups = floor(size(randIndexes,2) / 3);
randIndexes = randIndexes(1:(numIndexGroups*3));

while numIndexGroups < iterations
    randIndexes = [randIndexes rand(1,3)];
    numIndexGroups = floor(size(randIndexes,2) / 3);
end
randIndexes= randIndexes( randperm(size(randIndexes,2)) ); 
randIndexes = reshape(randIndexes,3,iterations); 
randIndexes = round(randIndexes .* (size(nonZinds,1)-1 ) + 1);
randIndexes = nonZinds(randIndexes);

for loop = 1:iterations

pointsIndex =  randIndexes(:,loop);

try 
if debugOn 
    fprintf('    loop %g\n', loop);
end


%
% FIND THE ELLIPSE CENTER 
%

% get some points we can look at the gradiant to find the tangent line
[I, J] = ind2sub(esize, pointsIndex(1) );  
point =[I, J]; 
[py, px] = find( (roipoly(e,point(2) + nc, point(1) + nr) & e) > 0  );
% find the tangent line
[P,S] = polyfit(px, py,1);

[I, J] = ind2sub( esize, pointsIndex(2));
point2 =[I, J];  
[py2, px2] = find( (roipoly(e,point2(2) + nc, point2(1) + nr) & e) > 0  );
[P2,S2] = polyfit(px2, py2,1);


[I, J]  = ind2sub(esize, pointsIndex(3) );
point3 = [I,J];
[py3, px3] = find( (roipoly(e,point3(2) + nc, point3(1) + nr) & e) > 0  );
[P3,S3] = polyfit(px3, py3,1);

% TANGENTs 1 and 2 
% get the line intercept of the tangent lines 
lines(1) = subs(lineeq, {m b}, {P(1) P(2)});
lines(2) = subs(lineeq, {m b}, {P2(1) P2(2)});
lineIntercept = solve(lines(1),lines(2));

% find the equation of the bisector
midx = ((point(2) + point2(2)) * 0.5);
midy = ((point(1) + point2(1)) * 0.5);
bisectSlope = (midy - lineIntercept.y) / (midx - lineIntercept.x ) ;
bisectB = lineIntercept.y - bisectSlope * lineIntercept.x;

% TANGENTs 2 and 3
% get the line intercept of the tangent lines 
lines2(1) = subs(lineeq, {m b}, {P2(1) P2(2)});
lines2(2) = subs(lineeq, {m b}, {P3(1) P3(2)});
lineIntercept2 = solve(lines2(1),lines2(2));

% find the equation of the bisector
midx2 = ((point2(2) + point3(2)) * 0.5);
midy2 = ((point2(1) + point3(1)) * 0.5);
bisectSlope2 = (midy2 - lineIntercept2.y) / (midx2 - lineIntercept2.x ) ;
bisectB2 = lineIntercept2.y -  bisectSlope2 * lineIntercept2.x ;

% find the center of the circle!
linesCent(1) = subs(lineeq, {m b}, {bisectSlope bisectB});
linesCent(2) = subs(lineeq, {m b}, {bisectSlope2 bisectB2});

lineInterceptCent = solve(linesCent(1),linesCent(2));
xCent = eval(lineInterceptCent.x);
yCent = eval(lineInterceptCent.y);

%
%  Draws the ellipse found on the current image( image - ellipses already 
%   found). Waits for a user to press enter before continueing. 
%
if showit 
    figure,imshow(e);
    % show the tangents
    hold on
    plot( 1:esize(2), [P(1)*(1:esize(2)) + P(2);...
                       P2(1)*(1:esize(2)) + P2(2); ...
                       P3(1)*(1:esize(2)) + P3(2)], 'LineWidth', 3); 
    hold on

    plot( 1:esize(2), [eval(bisectSlope*(1:esize(2)) + bisectB); ...
                      eval(bisectSlope2*(1:esize(2)) + bisectB2)], '--',...
                      'LineWidth', 3);
    hold on
    scatter(midx,midy,120,'gs','filled');
    scatter(midx2,midy2,120,'rs','filled');
    scatter(xCent,yCent, 200,'mp','filled');
    hold off
    input( 'Press enter to continue.');

end

% translate the ellipse to the origin

centerP = repmat([yCent xCent],[3 1] );  
transP = [point; point2; point3] - centerP;
fm(1) = subs(ellipseFcn, {x y}, {transP(1,2) transP(1,1)});
fm(2) = subs(ellipseFcn, {x y}, {transP(2,2) transP(2,1)});
fm(3) = subs(ellipseFcn, {x y}, {transP(3,2) transP(3,1)});
s = solve(fm(1),fm(2),fm(3));

% Check if this is an ellipse
check = eval(s.a*s.c - s.b^2);

if( yCent > 0 && xCent > 0 && check > 0   )
    majorAxis = eval( sqrt(abs(1/s.a))); % semimajor Axis
    minorAxis = eval( sqrt(abs(1/s.c))); % semiminor Axis
    if majorAxis > max(esize)|| minorAxis > max(esize) 
        if debugOn
            disp('Major or minor axis is greater then the image size')
        end
        continue;
    end
    
    score = 1; % default score is 1
    if debugOn 
        disp( 'Ellipse met check criteria' )
    end

    %
    % Check if the ellipse really exists in the image
    %
    % Genereate a mask to determine how many pixels lie on the perimeter 
    %   of the ellipse.
    % By getting a number of coordinates around the ellipse equal
    % to its circumference 
    % generating a mask
    %   And the mask with the image
    % checking if the resulting number of pixels are above a threshold
    %

    circumference = pi * ( majorAxis + minorAxis ); % this is approximate 
    [Xn, Yn] = ellipsep(majorAxis,minorAxis,0, xCent, yCent, ... 
        ceil(circumference));
    ellipseMask = zeros(esize);   
    rYn = round(Yn);
    rXn = round(Xn);

    %
    % Make sure the ellipse is fully within the image.
    % Assumption is all ellipeses are fully within the image
    % this can be changed by removing the points from (Xn,Yn) 
    % that are not in the image.
    %
    if  max(rYn) <= esize(1) && max(rXn) <= esize(2) ...
        && min(rYn) >= 0  && min(rXn) >= 0  
    
        Indn = sub2ind(esize,round(Yn),round(Xn));
    else
        if debugOn 
            disp('Ellipse is outside image: Yn or Xn to large or small');
        end
        continue;
    end 

    ellipseMask(Indn) = 1;
    pixelsFound = sum(sum(ellipseMask & e)); 

    if( pixelsFound  / circumference > ratioPixelsFoundToCircumferenceThresh )
        if debugOn 
            disp('Ellipse actually exists');
        end

        %
        % If the accumulator has elments then check if those
        % elements are similar to the current ellipse
        %
        accumulator = addToAccumulator(accumulator, xCent, yCent,  ... 
            majorAxis, minorAxis, score, distSimThresh, majorAxisSimThresh, ...
            minorAxisSimThresh, debugOn );

    end % if pixels found > ratio of pixels found  to circumference thresh 

end  % yCent xCent check

catch
    % 
    % Errors are generated each time through the loop for various reasons
    % Most of the time it is because of an inavlid elipse so we just
    % ignore and continue looking for ellipses.
    %
    if showError
    fprintf('Error caught ignoring it:\n %s\n', lasterr);
    end
end % try catch
end % for

% Check if any ellipses in the accumulator are best ellipses   
if( size(accumulator,1) > 0 )
    maxScorer = max(accumulator(:,5));
    if( maxScorer > scoreThreshHold )
        disp( '    Found an ellipse'); 

        bestScorers = find( accumulator(:,5) >= maxScorer );


        % add the best ellipse to the bestEllipses matrix
        % if an ellipse in the accumulator is similar to an
        % elllipse in the betsEllipses matrix we weight average 
        % them like we do when adding new ellipses to the accumulator
        for numBestInAccum = size(bestScorers,2) 

            bestEllipses = addToAccumulator(bestEllipses, ... 
                accumulator(numBestInAccum,1), ... % xCent  
                accumulator(numBestInAccum,2), ... % yCent
                accumulator(numBestInAccum,3), ... % majorAxis 
                accumulator(numBestInAccum,4), ... % minorAxis
                accumulator(numBestInAccum,5), ... % score
                distSimThresh,  ... 
                majorAxisSimThresh, minorAxisSimThresh, debugOn );
        end

        % we found an ellipse reset maxEpochs
        epochsWithoutFindingEllipse = 0;

        for bestInd = 1:size(bestScorers)
            % remove the ellipses we think we found from the image
            iPlot = bestScorers(bestInd); 
            circumference = pi * ( accumulator(iPlot,3)+ accumulator(iPlot,4));
            [Xbest, Ybest] =  ellipsep(accumulator(iPlot,3), ...
                    accumulator(iPlot,4), ...
                    0, ...
                    accumulator(iPlot,1), ... 
                    accumulator(iPlot,2), ...
                    circumference ); 
            ellipseMask = ones(esize);   
            Indn = sub2ind(size(e),floor(Ybest),floor(Xbest));
            ellipseMask(Indn) = 0;
            e = ellipseMask & e; 

        end

        % clear the accumulator for the next epoch
        accumulator = [];
        
        if debugOn 
            disp('The best ellipses');
            bestEllipses
        end
    end % if maxScorer
end % size accumulator 
end % while epochs

%
% Show the best ellipses on the original image if the user wants it
%
if size(bestEllipses,1) > 0
    if showBest  
        imshow(image)
        hold on
        for bestInd = 1:size(bestEllipses)
            iPlot = bestInd; 
            bestCircumference = pi * ( bestEllipses(iPlot,3)+ ...
                bestEllipses(iPlot,4));
            ellipsep(bestEllipses(iPlot,3), ...
                    bestEllipses(iPlot,4), ...
                    0, ...
                    bestEllipses(iPlot,1), ... 
                    bestEllipses(iPlot,2), ...
                    bestCircumference ); 
        end
        hold off
    end
else
    disp( 'No satisfactory ellipses found.');
end

warning on

%   Name:   addToAccumulator
%
%   Takes an accumulator and the paramters of a possible ellipse.
%       Determines if the possible ellipse is similar to any ellipse 
%       in the accumulator. If it is similar then it is averaged with the
%       the similar ellipses in the accumulator.
%       During averaging the ellipses in the accumulator are weighted
%       by their score so if twe have a really good ellipse it remains
%       relativily the same.
%       The new average ellipse replaces the exiting ellipse in the 
%       accumulator.
%       If the possible ellipse is not the same as an ellipse in the 
%       accumualtor it is just added to the end of the accumulator.
%
%   Arguments: many, all self explanatory
%
%   Returns: the old accumulator combined with new ellipses found
%
function accumulator = addToAccumulator(accumulator, xCent, yCent,  ... 
    majorAxis, minorAxis, score, distSimThresh, majorAxisSimThresh, ...
    minorAxisSimThresh, debugOn )

        % If the accumulator has elments then check if those
        % elements are similar to the current ellipse
        %

        if(size(accumulator,1) > 0 )

            % 
            % Compute differences with this ellipse and ellipses alread in
            % the accumulator
            %
            distance = sqrt( (accumulator(:,2) - yCent ).^2 + ...
                  (accumulator(:,1) - xCent ).^2 );
            majorAxisDiff = abs(accumulator(:,3) - majorAxis); 
            minorAxisDiff = abs(accumulator(:,4) - minorAxis); 

            %
            % check for similarities
            %
            similar = find( distance < distSimThresh & ...
                majorAxisDiff < majorAxisSimThresh & ...
                minorAxisDiff < minorAxisSimThresh );
            if( ~isempty(similar) )
                if debugOn
                    disp( 'Ellipse is similar to ellipse(s) in accumulator');
                 end
             % Found ellipses that are similar !
             % Calculate the new value by taking the weigheted average
             % by score of the current and new values.
                     
            simSize = size(similar,2);
             
             accumTemp =  (( accumulator(similar,1:4).* ...
               repmat(accumulator(similar, 5), [1,4 ]) ... % weighted accums 
               +  ...
               (repmat([ xCent yCent majorAxis minorAxis ] ... 
               .* score,...                         % weighed current value
               [simSize, 1 ]) ) ) ...  % sum of weighted values 
               ./ ... % divide weighted values by sum of weights 
               (repmat(accumulator(similar, 5) + score,[1,4]))); % sum of the 
                                                                     % weights  

              % attach old scores + new score and replace these values in
              % the accumulator
              accumulator(similar,:) = [accumTemp ...
                (accumulator(similar,5) + score )]; 
            else 
                % These values are not similar to the ones in the accumulator
                % so add them
                accumulator =  ...
                    [accumulator; xCent yCent majorAxis minorAxis score ];
            end  % if similar 
        else
            % this is an empty accumulator so add the values 
            accumulator = [ xCent yCent majorAxis minorAxis score ];
        end % if accumulator is empty


