function [segment_pts, coms, polygons] = segmentTracking_manual(im, com1, vect0, varargin)

%% segmentTracking returns tail as a combination of segments
%
%  What it does is start to locate the eyes and the beginning of the tail.
%  It then uses COMs to find fish axis (centers of mass for a different 
%  proportion of pixels). This allows to get the point at the beginning of
%  the tail. Then for each segment, segmentTracking assigns a polygon, and
%  check which pixels are in it. It computes center of mass of these
%  pixels, weighted by pixel value. A new segment is added using former
%  point and new center of mass.
%  NB: see Stytra paper by Vilim Stih et. al for more information.
%
%  Inputs:
%  - im [2D matrix]: image, with zebrafish, and its eyes visible.
%  - com1 [2D vector]: first center of mass.
%  - vect0 [2D vector]: normalized directional vector between first center
%    of mass and second center of mass.
%  - num_segments [integer, optional]: number of segments to use.
%  - tail_length [number, optional]: length of the tail.
%  - body_length [number, optional]: length between eyes and beginning of 
%    tail.
%  - inertia [number in (0, 1), optional]: in order to have segments that 
%    are more stable, the inertia is the proportion of the former segment
%    we want in the segment we are computing. This allows for a smoothing
%    of the tail representation.
%  - initial_box [number, optional]: length of initial box for tail
%    tracking.
%  - box_increment [number, optional]: increment for box length for each
%    new segment.
%
%  Outputs:
%  - segment_pts [2D matrix]: coordinates of segment points (first column
%    first coordinate, etc.).
%  - coms [2D matrix]: coordinates of the centers of mass found (first
%    column first coordinate, etc.).
%  - polygons [1D cell]: cell of 2D matrices containing coordinates of
%    polygons.
%
%  UPDATE 1: I had put in arguments flipud and fliplr, to modify
%  orientation of tail, but it was too complicated to use, so I replaced it
%  with flip, associated to a simple number in (1, 2, 3, 4), defining
%  orientation.
%  UPDATE 2: I added intertia to have a continuity in the tail across
%  segments. The higher it is, the more the segment considered will be
%  average with former segment orientation.
%  UPDATE 3: when the center of mass was too close to end of former
%  segments, the orientation could be very random for the new segment. In
%  order to make the center of mass further, I divided the initial polygon
%  into two, and computed a com for the two. The final com would be the
%  average of these two.


    %% Check inputs
    
    % Default values
    defaultNumSegs = 12;
    defaultTailLength = 80; % tail usually between 80 and 95 pixels
    defaultBodyLength = 35; % in pixels as well
    defaultInertia = 0;
    defaultInitialBox = 0.3;
    defaultBoxIncrement = 0.03;
    
    % Input parser
    p = inputParser;
    addRequired(p, 'im');
    addRequired(p, 'com1');
    addRequired(p, 'vect0');
    addOptional(p, 'num_segments', defaultNumSegs);
    addOptional(p, 'tail_length', defaultTailLength);
    addOptional(p, 'body_length', defaultBodyLength);
    addOptional(p, 'inertia', defaultInertia);
    addOptional(p, 'initial_box', defaultInitialBox);
    addOptional(p, 'box_increment', defaultBoxIncrement);
    parse(p, im, com1, vect0, varargin{:});
    
    
    %% Find beginnig of tail using eyes
    
    % First COM and directional vector
    com1 = p.Results.com1;
    vect0 = p.Results.vect0;
    
    % Get beginning of tail
    beginning_tail = com1 + p.Results.body_length*vect0;
    
    
    %% Parameters
    
    % Now finding tail segments
    segment_length = p.Results.tail_length / p.Results.num_segments;
    % Meshgrid for coordinates
    [Xt, Yt] = meshgrid(1:size(p.Results.im, 1), 1:size(p.Results.im, 2));
    Xt = Xt(:);
    Yt = Yt(:);
    
    % Output objects
    segment_pts = zeros(p.Results.num_segments+1, 2);
    segment_pts(1, :) = beginning_tail;
    coms = zeros(p.Results.num_segments, 2);
    polygons = cell(p.Results.num_segments, 1);
    
    
    %% Loop over the number of segments
    
    for i = 1:p.Results.num_segments
        
        % Define region of interest (aka polygons)
        pt1 = segment_pts(i, :) + (p.Results.initial_box + p.Results.box_increment*i) * segment_length * [vect0(2), -vect0(1)];
        pt2 = pt1 + 0.5 * segment_length * vect0;
        pt4 = segment_pts(i, :) + (p.Results.initial_box + p.Results.box_increment*i) * segment_length * [-vect0(2), vect0(1)];
        pt3 = pt4 + 0.5 * segment_length * vect0;
        polydef = [pt1; pt2; pt3; pt4; pt1];
        pt5 = pt2 + 0.5 * segment_length * vect0;
        pt6 = pt3 + 0.5 * segment_length * vect0;
        polydef2 = [pt2; pt5; pt6; pt3; pt2];
        polygons{i} = [pt1; pt5; pt6; pt4; pt1];
        
        % Get points inside region
        pts_in_region = inpolygon(Xt, Yt, polydef(:, 1), polydef(:, 2));   
        pts_to_keep = sub2ind(size(p.Results.im), Xt(pts_in_region), Yt(pts_in_region));
        im_vals = double(p.Results.im(pts_to_keep));
        im_vals = (im_vals - min(im_vals) + 1).^4;
        pts_in_region2 = inpolygon(Xt, Yt, polydef2(:, 1), polydef2(:, 2));   
        pts_to_keep2 = sub2ind(size(p.Results.im), Xt(pts_in_region2), Yt(pts_in_region2));
        im_vals2 = double(p.Results.im(pts_to_keep2));
        im_vals2 = (im_vals2 - min(im_vals2) + 1).^4;
        
        % Find center of mass
        com = [im_vals' * Xt(pts_in_region) / sum(im_vals), ...
               im_vals' * Yt(pts_in_region) / sum(im_vals)]; % NB center of mass focuses on pixel number, not on pixel value
        com2 = [im_vals2' * Xt(pts_in_region2) / sum(im_vals2), ...
               im_vals2' * Yt(pts_in_region2) / sum(im_vals2)];
        com_mean = (com + com2) / 2;
        coms(i, :) = com_mean;
        
        % Find vector, and end of new segment
        vect = com_mean - segment_pts(i, :);
        vectn = vect ./ sqrt(sum(vect.^2)); % vect, normalized
        vect0 = (1 - p.Results.inertia) * vectn + p.Results.inertia * vect0;
        vect0 = vect0 ./ sqrt(sum(vect0.^2)); % vect0, normalized
        segment_pts(i+1, :) = segment_pts(i, :) + segment_length * vect0;
        
    end
    
    
end