function [angles, angle0, pt0] = getTrackingAngles(segment_pts)

%% getTrackingAngles returns angles based on segment positions
%
%  Based on the points provided by segment_pts, it infers a tail
%  orientation, and provides a vector of angles, associated to the angle
%  difference between a segment and the segment before it. 
%
%  Inputs:
%  - segment_pts [2D matrix]: matrix including coordinates for each segment
%    point. First column is first coordinates, second is second coordinate.
%
%  Outputs:
%  - angles [column vector]: vector containing angles. If there are n
%    segments, angles will be of length n-1. !!!Angles are in degrees!!!
%    NB: in this case, segment_pts will have n+1 points.
%  - angle0 [number]: first segment angle. Be careful, although the other 
%    angles represent a difference between two segments, this one is 
%    absolute. !!! Angle is in degrees!!!
%  - pt0 [row vector]: vector containing coordinates of beginning of first
%    segment, if necessary.


    %% Correcting orientation with coordinates rotations
    
    % Get first segment orientation
    dx = segment_pts(2, 1) - segment_pts(1, 1);
    dy = segment_pts(2, 2) - segment_pts(1, 2);
    
    % Correct orientation in necessary
    if abs(dy) <= abs(dx) && dx < 0
        segment_pts = segment_pts * [-1, 0; 0, -1];
    elseif abs(dx) <= abs(dy) && dy < 0
        segment_pts = segment_pts * [0, 1; -1, 0];
    elseif abs(dx) <= abs(dy) && dy > 0
        segment_pts = segment_pts * [0, -1; 1, 0];
    end
    
    
    
    %% Deducing angles
    
    % Get first segment point
    pt0 = segment_pts(1, :);
    
    % And get angles from corrected segments
    angles = segment_pts(2:end, :) - segment_pts(1:end-1, :);
    angles = atan(angles(:, 2) ./ angles(:, 1));
    angle0 = (180 / pi) * angles(1); % first absolute angle in degrees
    angles = angles(2:end) - angles(1:end-1);
    angles = (180 / pi) * angles; % converting to degrees
        

end