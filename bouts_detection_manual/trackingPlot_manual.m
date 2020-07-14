function trackingPlot_manual(path_to_video, varargin)

%% trackingPlot simply plots the live tracking of the tail
%
%  For each frame in the video, boutsWrapper launches an analysis to detect
%  tail segments, using segmentTracking. It then displays on screen the
%  image and the segments.
%
%  Inputs: see segmentTracking.
%  - frame_pause [number, optional]: time difference between two displayed
%    frames.
%
%  Outputs:
%  - tracking [structure]: structure containing all the information.


    %% Check inputs
    
    % Default values
    defaultNumSegs = 12;
    defaultTailLength = 80; % tail usually between 80 and 95 pixels
    defaultBodyLength = 35; % in pixels as well
    defaultInertia = 0;
    defaultInitialBox = 0.3;
    defaultBoxIncrement = 0.03;
    defaultFramePause = 0.1;
    
    % Input parser
    p = inputParser;
    addRequired(p, 'path_to_video');
    addOptional(p, 'num_segments', defaultNumSegs);
    addOptional(p, 'inertia', defaultInertia);
    addOptional(p, 'body_length', defaultBodyLength);
    addOptional(p, 'tail_length', defaultTailLength);
    addOptional(p, 'initial_box', defaultInitialBox);
    addOptional(p, 'box_increment', defaultBoxIncrement);
    addOptional(p, 'frame_pause', defaultFramePause);
    parse(p, path_to_video, varargin{:});
    
    
    %% Specify manually the two points allowing tail tracking
    
    % Plot first frame of video
    fig = figure;
    vid = VideoReader(p.Results.path_to_video);
    im = readFrame(vid);
    im = mean(im, 3);
    image(im, 'CDataMapping', 'scaled')
    axis equal
    
    % Get the two points from user
    fprintf('Select first point on fish tail \n');
    [x_1, y_1] = getpts(fig);
    fprintf('Select second point on fish tail \n');
    [x_2, y_2] = getpts(fig);
    
    % Compute first center of mass and initial vector
    com1 = [y_1, x_1];
    vect0 = [y_2-y_1, x_2-x_1];
    vect0 = vect0 ./ sqrt(sum(vect0.^2));


    %% Create videoreader and get number of frames
    
    vid = VideoReader(p.Results.path_to_video);
    numframes = floor(vid.Duration * vid.FrameRate);
    
    
    %% Plot at every frame
    
    % Loop
    myfig = figure();
    for i = 1:numframes
        % Read frame and analyse it
        im = readFrame(vid);
        im = mean(im, 3);
        [segment_pts, coms, polygons] = segmentTracking_manual(im, com1, vect0, 'num_segments', p.Results.num_segments, 'inertia', p.Results.inertia, ...
                                                               'body_length', p.Results.body_length, 'tail_length', p.Results.tail_length, 'initial_box', ...
                                                               p.Results.initial_box, 'box_increment', p.Results.box_increment);                           
%         clf(myfig);
        hold on
        image(im, 'CDataMapping', 'scaled')
        plot(coms(:, 2), coms(:, 1), 'o')
        plot(segment_pts(:, 2), segment_pts(:, 1), 'r')
        for j = 1:length(polygons)
            plot(polygons{j}(:, 2), polygons{j}(:, 1), 'g')
        end
        axis equal
        hold off
        title(i)
        pause(p.Results.frame_pause)
        drawnow limitrate
    end


end