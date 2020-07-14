function tracking = boutsWrapper(path_to_video, varargin)

%% boutsWrapper simply combines functions to get bouts from video
%
%  For each frame in the video, boutsWrapper launches an analysis to detect
%  bouts, using segmentTracking in the first place, combined with
%  getTrackingAngles. Finally, it uses detectBouts to obtain final bouts,
%  and bouts intensity (see detectBouts).
%
%  Inputs: see three different functions.
%
%  Outputs:
%  - tracking [structure]: structure containing all the information.


    %% Check inputs
    
    % Default values
    defaultNumSegs = 12;
    defaultTailLength = 80; % tail usually between 80 and 95 pixels
    defaultBodyLength = 35; % in pixels as well
    defaultInertia = 0;
    defaultNumPix1 = 100; % number of pixels for COM 1
    defaultNumPix2 = 500; % number of pixels for COM 2
    defaultInitialBox = 0.3;
    defaultBoxIncrement = 0.03;
    defaultTriggerValue = 9;
    defaultNumberOfPointsAfter = 9;
    
    % Input parser
    p = inputParser;
    addRequired(p, 'path_to_video');
    addOptional(p, 'num_segments', defaultNumSegs);
    addOptional(p, 'inertia', defaultInertia);
    addOptional(p, 'body_length', defaultBodyLength);
    addOptional(p, 'tail_length', defaultTailLength);
    addOptional(p, 'initial_box', defaultInitialBox);
    addOptional(p, 'box_increment', defaultBoxIncrement);
    addOptional(p, 'num_pix1', defaultNumPix1);
    addOptional(p, 'num_pix2', defaultNumPix2);
    addOptional(p, 'trigger_value', defaultTriggerValue);
    addOptional(p, 'num_pts_after', defaultNumberOfPointsAfter);
    parse(p, path_to_video, varargin{:});


    %% Create videoreader and get number of frames
    
    vid = VideoReader(p.Results.path_to_video);
    numframes = floor(vid.Duration * vid.FrameRate);
    
    
    %% Save inputs in parameters structure
    
    parameters = struct;
    parameters.numframes = numframes;

    parameters.path_to_video = p.Results.path_to_video;
    parameters.num_segments = p.Results.num_segments;
    parameters.inertia = p.Results.inertia;
    parameters.body_length = p.Results.body_length;
    parameters.tail_length = p.Results.tail_length;
    parameters.initial_box = p.Results.initial_box;
    parameters.box_increment = p.Results.box_increment;
    parameters.num_pix1 = p.Results.num_pix1;
    parameters.num_pix2 = p.Results.num_pix2;
    parameters.trigger_value = p.Results.trigger_value;
    parameters.num_pts_after = p.Results.num_pts_after;
    
    
    %% Get angles at every frame
    
    % Outputs of frame analysis
    Segment_pts_x = zeros(parameters.numframes, parameters.num_segments+1);
    Segment_pts_y = zeros(parameters.numframes, parameters.num_segments+1);
    Angles = zeros(parameters.numframes, parameters.num_segments-1);
    Angle0 = zeros(parameters.numframes, 1);
    
    % Loop
    for i = 1:parameters.numframes
        % Read frame and analyse it
        im = readFrame(vid);
        im = mean(im, 3);
        segment_pts = segmentTracking(im, 'num_segments', parameters.num_segments, 'inertia', parameters.inertia, 'body_length', parameters.body_length, ...
                                      'tail_length', parameters.tail_length, 'initial_box', parameters.initial_box,  'box_increment', parameters.box_increment, ...
                                      'num_pix1', parameters.num_pix1, 'num_pix2', parameters.num_pix2);
        % If tail leaves screen, there is a nan value, let's correct it
        if sum(isnan(segment_pts(:, 1))) ~= 0 && i > 2
            for ind = find(isnan(segment_pts(:, 1)))
                segment_pts(ind, 1) = 2 * Segment_pts_x(i-1, ind)' - Segment_pts_x(i-2, ind)';
                segment_pts(ind, 2) = 2 * Segment_pts_y(i-1, ind)' - Segment_pts_y(i-2, ind)';
            end
        end
        % Save positions
        Segment_pts_x(i, :) = segment_pts(:, 1)';
        Segment_pts_y(i, :) = segment_pts(:, 2)';
        % Get angles
        [angles, angle0] = getTrackingAngles(segment_pts);
        Angles(i, :) = angles';
        Angle0(i) = angle0;
    end
    
    
    %% Finally detect bouts
    
    % Compute total angle
    total_angle = sum([Angle0, Angles], 2);
    
    % Detect bouts using total angle
    [bouts, bouts_intensity, bouts_initial] = detectBouts(total_angle, 'trigger_value', parameters.trigger_value, 'num_pts_after', parameters.num_pts_after);
    
    
    %% Returns output as a structure
    
    tracking = struct;
    tracking.parameters = parameters;
    tracking.Segment_pts_x = Segment_pts_x;
    tracking.Segment_pts_y = Segment_pts_y;
    tracking.Angles = Angles;
    tracking.Angle0 = Angle0;
    tracking.total_angle = total_angle;
    tracking.bouts = bouts;
    tracking.bouts_intensity = bouts_intensity;
    tracking.bouts_initial = bouts_initial;


end