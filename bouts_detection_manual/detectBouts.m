function [bouts, bouts_intensity, bouts_initial] = detectBouts(total_angle, varargin)

%% detectBouts returns bouts detected using total angle along experiment
%
%  Total angle is first differenciated. Then the differential is
%  thresholded, and this returns a first set of points for which there is a
%  high tail movement. Using total angle, we integrate a certain number of
%  points after each point that was detected using thresholding. If a
%  thresholded point is detected within this number of points after a point
%  that was already detected, we consider it to be the same bout. That we
%  take the initial point for each accumulation, and this is the final bout
%  we return.
%
%  Inputs:
%  - total_angle [1D array]: angle between initial point and end of tail.
%  - trigger_value [number, optional]: value at which we consider the
%    differential of total_angle to be a bout, or part of a bout.
%  - num_pts_after [number, optional]: number of points after a bout is
%    detected on which we integrate to have bout intensity.
%
%  Outputs:
%  - bouts [1D array of booleans]: final bouts detected.
%  - bouts_intensity [1D array]: integration of total_angle for each bout
%    detected in bouts_initial.
%  - bouts_initial [1D array of booleans]: initial points in the
%    differential of total_angle that were above threshold value.


    %% Check inputs
    
    % Default values
    defaultTriggerValue = 9;
    defaultNumberOfPointsAfter = 9;
    
    % Input parser
    p = inputParser;
    addRequired(p, 'total_angle');
    addOptional(p, 'trigger_value', defaultTriggerValue);
    addOptional(p, 'num_pts_after', defaultNumberOfPointsAfter);
    parse(p, total_angle, varargin{:});
    
    
    %% Differenciate and find points above threshold
    
    diff_angle = diff(total_angle);
    bouts_initial = (abs(diff_angle) > p.Results.trigger_value);
    
    
    %% Integrate for each point above trigger
    
    last_trigger = 0;
    index = 1;
    bouts_intensity = zeros(size(total_angle));
    while index <= length(diff_angle)
        % Define reference angle if new trigger
        if bouts_initial(index) && last_trigger == 0
            former_angle = total_angle(index);
        end
        % Compute integration
        if bouts_initial(index)
            bouts_intensity(index+1) = bouts_intensity(index) + abs(total_angle(index+1)-former_angle);
            last_trigger = p.Results.num_pts_after;
        elseif last_trigger > 0
            bouts_intensity(index+1) = bouts_intensity(index) + abs(total_angle(index+1)-former_angle);
            last_trigger = max([0, last_trigger-1]);
        end
        index = index + 1;
    end
    
    
    %% Finally get actual bouts
    
    bouts = zeros(size(bouts_intensity));
    for i = 2:length(bouts)
        if bouts_intensity(i-1) == 0 && bouts_intensity(i) ~= 0
            bouts(i) = 1;
        end
    end
    
    
end