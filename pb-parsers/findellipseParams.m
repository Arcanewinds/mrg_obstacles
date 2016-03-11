% Contains all the parameters for the ellipse detection program findellipse.m
% Edit this file directly to change the parameters.
%
% This file is read by findellipse.m before it begins.
%
% Author:   Samuel Inverso 
% File:     $Id: findellipseParams.m,v 1.2 2002/05/16 17:54:40 sai6189 Exp sai6189 $
%
% License:
% Use this code freely, it is distributed without warranty express or 
% implied.



% 
% If an ellipse's score in the accumulator is > scoreThreshold it is 
% considered  ellipse in the image. 
%
scoreThreshHold = 0; 

%
% Numbers of times through the loop accumulating potential ellipses
%
iterations = 50; 

%
% The total number of epochs to run through.
% An epoch is the time in the algorithm it attempts to find an ellipse.
% Each epoch runs a loops for the number of 'iterations' set above.
%   Only the best ellipses in each epoch are saved. 
%
% Note: the algorithm may stop prematruely if maxEpochsWithoutFIndingElipse
% is set to a low number.
%
maxEpochs =10; 

%
% The maximum number of consecutive epochs we will go through before
% prematurely terminating the algorithm
%
maxEpochsWithoutFindingEllipse = 20;


%
% The check to determine if an ellipse is really in the image
% A number of pixels equal to the circumference of the ellipse 
% that would be on the perimeter of the ellipse are extracted from
% the image. If the nubmer of pixels divided by the circumference
% of the circle is greater than the rationPixelsFoundToCircumferenceThresh 
% it is considered to exist 
%
ratioPixelsFoundToCircumferenceThresh = 0.2; 

% 
% If the distance between an accumulated ellipse and the ellipse
% being added to the accumulator is < distSimThresh 
% and
% the absolute difference between the major axises is < majorAxisSimThresh
% and
% the absolute difference between the minor axises is < minorAxisSimThresh 
% 
% then the ellipse in the accumulator is close enough the ellipse
% being tested that they are average together. The accumulted ellipsed
% is weighted by its score in the accumualtor.
%
distSimThresh = 6 ;
majorAxisSimThresh = 5;
minorAxisSimThresh = 5;

% 
% Any error that occurs during accumulation is ignored because it
% most likely occured because the ellipse was not in the image.
% All error messages are suppressed unless showError = 1.
% Note: if set to 1 the program still ignores the error, but it
% will tell you it ignores it ;-)
%
showError = 0;

% 1 to show the best ellipses on the image at the end. 0 don't do it
showBest = 1;

% 1 to print debugging info, 0 to not
debugOn = 0;

% 1 to show the accumulated ellipse, 0 to not
showit = 0; % show the  

