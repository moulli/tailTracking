function [segment_pts, coms, polygons] = segmentTracking_OLD(im, varargin)

%% segmentTracking returns tail as a combination of segments
%
%  What it does is start to locate the eyes and the beginning of the tail.
%  It then uses PCA to find fish axis (the principal component of the
%  pixels located before). This allows to get the point at the beginning of
%  the tail. Then for each segment, segmentTracking assigns a polygon, and
%  check which pixels are in it. It computes center of mass of these
%  pixels, weighted by pixel value. A new segment is added using former
%  point and new center of mass.
%  NB: see Stytra paper by Vilim Stih et. al for more information.
%
%  Inputs:
%  - im [2D matrix]: image, with zebrafish, and its eyes visible.
%  - flip [(1, 2, 3 or 4), optional]: sometimes the image requires
%    to change its orientation, this changes it from the four possible.
%  - num_segments [integer, optional]: number of segments to use.
%  - tail_length [number, optional]: length of the tail.
%  - body_length [number, optional]: length between eyes and beginning of 
%    tail.
%  - inertia [number in (0, 1), optional]: in order to have segments that 
%    are more stable, the inertia is the proportion of the former segment
%    we want in the segment we are computing. This allows for a smoothing
%    of the tail representation.
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
    defaultFlip = 1;
    defaultNumSegs = 12;
    defaultTailLength = 80; % tail usually between 80 and 95 pixels
    defaultBodyLength = 35; % in pixels as well
    defaultInertia = 0;
    defaultNumPixelsPCA = 190; % number of pixels for PCA
    
    % Input parser
    p = inputParser;
    addRequired(p, 'im');
    addOptional(p, 'flip', defaultFlip);
    addOptional(p, 'num_segments', defaultNumSegs);
    addOptional(p, 'tail_length', defaultTailLength);
    addOptional(p, 'body_length', defaultBodyLength);
    addOptional(p, 'inertia', defaultInertia);
    addOptional(p, 'num_pixels_pca', defaultNumPixelsPCA);
    parse(p, im, varargin{:});
    
    
    %% Find beginnig of tail using eyes
    
    % Find fish axis using PCA and proportion of lowest pixels
    [fx, fy] = ind2sub(size(p.Results.im), find(p.Results.im < quantile(p.Results.im(:), p.Results.num_pixels_pca/numel(p.Results.im))));
    pca0 = pca([fx, fy]);
    vect0 = [-pca0(1, 1), pca0(1, 2)];
    if p.Results.flip == 2
        vect0(1) = -vect0(1);
    elseif p.Results.flip == 3
        vect0(2) = -vect0(2);
    elseif p.Results.flip == 4
        vect0(1) = -vect0(1);
        vect0(2) = -vect0(2);
    end

    % Find beginning of tail using proportion of lowest pixels
    [fxe, fye] = ind2sub(size(p.Results.im), find(p.Results.im < quantile(p.Results.im(:), p.Results.num_pixels_pca/numel(p.Results.im))));
    mid_eyes = mean([fxe, fye]);
    beginning_tail = mid_eyes + p.Results.body_length*vect0;
    
    % NB: it is possible to have a different approach, where instead of
    % using a proportion of pixels, with use an absolute value of pixels
    % (eyes represent 100 to 140 pixels), and body around 360 pixels.
    % 1500 pixels should work fine as well...
    % Although it might sound better, in practice I did not have it
    % working:
    % [~, sortim] = sort(im(:));
    % [fx, fy] = ind2sub(size(p.Results.im), sortim(1:npixels));
    
    
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
        pt1 = segment_pts(i, :) + (0.3 + 0.03*i) * segment_length * [vect0(2), -vect0(1)];
        pt2 = pt1 + 0.5 * segment_length * vect0;
        pt4 = segment_pts(i, :) + (0.3 + 0.03*i) * segment_length * [-vect0(2), vect0(1)];
        pt3 = pt4 + 0.5 * segment_length * vect0;
        polydef = [pt1; pt2; pt3; pt4; pt1];
        pt5 = pt2 + 0.5 * segment_length * vect0;
        pt6 = pt3 + 0.5 * segment_length * vect0;
        polydef2 = [pt2; pt5; pt6; pt3; pt2];
        polygons{i} = [pt1; pt5; pt6; pt4; pt1];
        
        % Get points inside region
        pts_in_region = inpolygon(Xt, Yt, polydef(:, 1), polydef(:, 2));   
        pts_to_keep = sub2ind(size(p.Results.im), Xt(pts_in_region), Yt(pts_in_region));
        im_vals = (256 - double(p.Results.im(pts_to_keep))).^4;
        pts_in_region2 = inpolygon(Xt, Yt, polydef2(:, 1), polydef2(:, 2));   
        pts_to_keep2 = sub2ind(size(p.Results.im), Xt(pts_in_region2), Yt(pts_in_region2));
        im_vals2 = (256 - double(p.Results.im(pts_to_keep2))).^4;
        
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