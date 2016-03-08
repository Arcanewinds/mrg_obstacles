function [ poses ] = GetVisualOdometry( varargin )
%[ poses ] GetVisualOdometry ( moos_mailbox,
%                              moos_channel,
%                              only_newest [=FALSE])
%
% INPUTS:
%   moos_mailbox: MOOS mailbox structure array returned from calling
%      mexmoos('FETCH').
%   moos_channel: String containing the name of the MOOS channel (variable)
%      for which to return the messages (if one exists).
%   only_newest: Only return the latest message to be received on the
%      channel specified by moos_channel (default = FALSE).
%
% OUTPUTS:
%   poses: Struct array of world-frame x, y, theta poses (or EMPTY if there 
%      are no poses available).

% Michael Tanner
% March 2016
% Mobile Robotics Group, Oxford University.

%% Parse inputs
arg_idx = 1;
if (~(isempty(varargin{arg_idx}) || ...
        (isstruct(varargin{arg_idx}) && ...
        all(isfield(varargin{arg_idx}, {'KEY', 'TIME', 'BIN'})) )))
    error(['%s - moos_mailbox must be a struct array output from ' ...
        'mexmoos(''FETCH''), or empty.'], mfilename);
end
moos_mailbox = varargin{arg_idx};
arg_idx = arg_idx + 1;

if (~(ischar(varargin{arg_idx}) && ~isempty(varargin{arg_idx})))
    error('%s - moos_channel must be a non-empty string.', mfilename);
end
moos_channel = varargin{arg_idx};
arg_idx = arg_idx + 1;

% parse optional inputs
only_newest = false;
if ( nargin >= arg_idx )
    if ( ~islogical(varargin{arg_idx}) )
        error('%s - only_newest must be a logical value (true or false)',...
            mfilename);
    end
    only_newest = varargin{arg_idx};
end

%% Get world poses
transforms = GetRelativePoses(moos_mailbox, moos_channel, only_newest);

%% Convert to X, Y, Theta struct array
poses = cell(size(transforms));
sigma_xyz = 0.0; % metres
sigma_rpy = 0.0; % radians
for i = 1:max(size(transforms))
    noise = [sigma_xyz * randn([1 3]), sigma_rpy * randn([1 3])];
    xyzrpy = noise + transforms(i).xyzrpy;
    
    poses{i}.timestamp = transforms(i).src_timestamp;
    poses{i}.x = xyzrpy(1);
    poses{i}.y = xyzrpy(2);
    poses{i}.theta = xyzrpy(6);
end

end
