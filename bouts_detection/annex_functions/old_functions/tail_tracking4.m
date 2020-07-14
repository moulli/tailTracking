clear; close all; clc


%% Recover all bouts for tilt and roll

% Folder path and tilt/roll indication
folder_path = '/home/ljp/Science/balancoire_data/2020-02';
tilt_folders = [10, 12, 13, 18];
roll_folders = [19, 20];

% Final matrices
params = 4;
tilt = cell(0, params);
roll = cell(0, params);

% Loop over the files
for folder = dir(folder_path)'
    % Delete first two folders
    if folder.name == "." || folder.name == ".."
        continue
    end
    % Create temporary value vector
    values = cell(1, params);
    values{1} = folder.name;
    % Path to fish
    fish_path = fullfile(folder_path, folder.name);
    % Loop over fish
    for fish = dir(fish_path)'
        % Delete first two folders
        if fish.name == "." || fish.name == ".."
            continue
        end
        values{2} = fish.name;
        % Path to run
        run_path = fullfile(fish_path, fish.name);
        % Loop over run
        for run = dir(run_path)'
            % Delete first two folders
            if run.name == "." || run.name == ".."
                continue
            end
            values{3} = run.name;
            
            % Create vector to save later
            bouts = [];
            % Loop over files
            file_path = fullfile(run_path, run.name);
            for file = dir(file_path)'
                if endsWith(file.name, '.avi')
                    % Build video path
                    video_path = fullfile(file_path, file.name);
                    % Set parameters
                    num_segments = 10;
                    inertia = 0.6;
                    body_length = 0;
                    tail_length = 115;
                    initial_box = 0.3;
                    box_increment = 0.03;
                    % Analyze video
                    tracking = boutsWrapper(video_path, 'num_segments', num_segments, 'inertia', inertia, 'body_length', body_length, 'tail_length', ...
                                            tail_length, 'initial_box', initial_box, 'box_increment', box_increment);
                    % Save video
                    save_path = strcat(video_path(1:end-4), '_tracking.mat');
                    save(save_path, 'tracking')
                    % Save number of bouts
                    num_bouts = sum(tracking.bouts);
                    bouts = cat(2, bouts, num_bouts);
                end
            end        
            % Save data
            values{4} = bouts;
            % Tilt
            if any(ismember(tilt_folders, str2double(folder.name)))
                tilt = [tilt; values];
            end
            % Roll
            if any(ismember(roll_folders, str2double(folder.name)))
                roll = [roll; values];
            end
            % Display information
            disp(file_path)
        end
    end
end


%% Statistics on tilt

tilt_base = zeros(0, 1);
tilt_vest1 = zeros(0, 1);
tilt_vest2 = zeros(0, 1);
tilt_vest3 = zeros(0, 1);
for ex = 1:size(tilt, 1)
    bouts = tilt{ex, 4};
    if length(bouts) == 3
        tilt_vest1 = cat(1, tilt_vest1, bouts(1));
        tilt_vest2 = cat(1, tilt_vest2, bouts(2));
        tilt_vest3 = cat(1, tilt_vest3, bouts(3));
    elseif length(bouts) == 4
        tilt_base = cat(1, tilt_base, bouts(1));
        tilt_vest1 = cat(1, tilt_vest1, bouts(2));
        tilt_vest2 = cat(1, tilt_vest2, bouts(3));
        tilt_vest3 = cat(1, tilt_vest3, bouts(4));
    end
end
tilt_quantile = zeros(4, 10);
for i = 1:10
    tilt_quantile(1, i) = quantile(tilt_base, 0.1*i);
    tilt_quantile(2, i) = quantile(tilt_vest1, 0.1*i);
    tilt_quantile(3, i) = quantile(tilt_vest2, 0.1*i);
    tilt_quantile(4, i) = quantile(tilt_vest3, 0.1*i);
end

% boxplots
tilt_groups = [ones(size(tilt_base));
                2*ones(size(tilt_vest1));
                3*ones(size(tilt_vest2));
                4*ones(size(tilt_vest3))];
tilt_boxplot = [tilt_base;
                tilt_vest1;
                tilt_vest2;
                tilt_vest3];
figure
boxplot(tilt_boxplot, tilt_groups)
xticklabels({'baseline'; 'reference gain'; 'high gain'; 'low gain'})
title('Boxplot of bouts distribution for tilt protocol', 'Interpreter', 'latex')


%% Statistics on roll

roll_base = zeros(0, 1);
roll_vest1 = zeros(0, 1);
roll_vest2 = zeros(0, 1);
roll_vest3 = zeros(0, 1);
for ex = 1:size(roll, 1)
    bouts = roll{ex, 4};
    if length(bouts) == 3
        roll_vest1 = cat(1, roll_vest1, bouts(1));
        roll_vest2 = cat(1, roll_vest2, bouts(2));
        roll_vest3 = cat(1, roll_vest3, bouts(3));
    elseif length(bouts) == 4
        roll_base = cat(1, roll_base, bouts(1));
        roll_vest1 = cat(1, roll_vest1, bouts(2));
        roll_vest2 = cat(1, roll_vest2, bouts(3));
        roll_vest3 = cat(1, roll_vest3, bouts(4));
    end
end
roll_quantile = zeros(4, 10);
for i = 1:10
    roll_quantile(1, i) = quantile(roll_base, 0.1*i);
    roll_quantile(2, i) = quantile(roll_vest1, 0.1*i);
    roll_quantile(3, i) = quantile(roll_vest2, 0.1*i);
    roll_quantile(4, i) = quantile(roll_vest3, 0.1*i);
end

% boxplots
roll_groups = [ones(size(roll_base));
                2*ones(size(roll_vest1));
                3*ones(size(roll_vest2));
                4*ones(size(roll_vest3))];
roll_boxplot = [roll_base;
                roll_vest1;
                roll_vest2;
                roll_vest3];
figure
boxplot(roll_boxplot, roll_groups)
xticklabels({'baseline'; 'reference gain'; 'high gain'; 'low gain'})
title('Boxplot of bouts distribution for roll protocol', 'Interpreter', 'latex')

    

%% Experimenting new segmentTracking and creating function

vid_path = '/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run2/1-2-vestibular.avi';

function num_bouts = analyzeVideo(vid_path)

    vid = VideoReader(vid_path);

    parameters = struct;
    parameters.numframes = floor(vid.Duration * vid.FrameRate);

    parameters.num_segments = 12;
    parameters.inertia = 0.5;
    parameters.body_length = 0;
    parameters.tail_length = 125;
    parameters.initial_box = 0.5;
    parameters.box_increment = 0.04;
    Segment_pts_x = zeros(parameters.numframes, parameters.num_segments+1);
    Segment_pts_y = zeros(parameters.numframes, parameters.num_segments+1);
    Angles = zeros(parameters.numframes, parameters.num_segments-1);
    Angle0 = zeros(parameters.numframes, 1);

    for i = 1:parameters.numframes
        % Read frame and analyse it
        im = readFrame(vid);
        im = mean(im, 3);
        segment_pts = segmentTracking(im, 'num_segments', parameters.num_segments, 'inertia', parameters.inertia, 'body_length', parameters.body_length, ...
                                      'tail_length', parameters.tail_length, 'initial_box', parameters.initial_box,  'box_increment', parameters.box_increment);
        % Save positions
        Segment_pts_x(i, :) = segment_pts(:, 1)';
        Segment_pts_y(i, :) = segment_pts(:, 2)';
        % Get angles
        [angles, angle0] = getTrackingAngles(segment_pts);
        Angles(i, :) = angles';
        Angle0(i) = angle0;
    end

    total_angle = sum([Angle0, Angles], 2);
    [bouts, bouts_intensity, bouts_initial] = detectBouts(total_angle);

    tracking = struct;
    tracking.parameters = parameters;
    tracking.Segment_pts_x = Segment_pts_x;
    tracking.Segment_pts_y = Segment_pts_y;
    tracking.Angles = Angles;
    tracking.Angle0 = Angle0;
    tracking.bouts = bouts;
    tracking.bouts_intensity = bouts_intensity;
    tracking.bouts_initial = bouts_initial;

    save_path = strcat(vid_path(1:end-4), '_tracking.mat');
    save(save_path, 'tracking')
    
    num_bouts = sum(tracking.bouts);

end