clear; close all; clc


%% Load video

vid_path = '/home/ljp/Desktop/video.avi';
vid = VideoReader(vid_path);


%% Detect tail for first image

% Get image and average it across RGB channels
im = readFrame(vid);
figure; imshow(im); axis equal
im = mean(im, 3);
figure; image(im, 'CDataMapping', 'scaled'); axis equal

% Get lowest pixels
num_pix = 100;
f_ind = find(im <= quantile(im(:), num_pix/numel(im)));
min_pix = zeros(size(im)); min_pix(f_ind) = 1; figure; image(min_pix, 'CDataMapping', 'scaled'); axis equal
[fx, fy] = ind2sub(size(im), f_ind);

% Get principal components for lowest pixels
fpca = pca([fx, fy]);


%% Problem is PCA has no orientation, let's try with 2 COMs

% First COM
num_pix1 = 100;
f_ind1 = find(im <= quantile(im(:), num_pix1/numel(im)));
[fx1, fy1] = ind2sub(size(im), f_ind1);
com1 = mean([fx1, fy1]);

% Second COM
num_pix2 = 500;
f_ind2 = find(im <= quantile(im(:), num_pix2/numel(im)));
[fx2, fy2] = ind2sub(size(im), f_ind2);
com2 = mean([fx2, fy2]);

% Get direction vector and average it
dvect = com2 - com1;
dvectn = dvect ./ sqrt(sum(dvect.^2));


%% Ok let's try our modified function

num_segments = 10;
[segment_pts, coms, polygons] = segmentTracking(im, 'num_segments', num_segments, 'inertia', 0.2, 'body_length', 0, 'tail_length', 120, 'initial_box', 0.5);
figure
hold on
image(im, 'CDataMapping', 'scaled')
plot(coms(:, 2), coms(:, 1), 'o')
plot(segment_pts(:, 2), segment_pts(:, 1), 'r')
for j = 1:length(polygons)
    plot(polygons{j}(:, 2), polygons{j}(:, 1), 'g')
end
axis equal


%% Ok now that it works let's try and get it for all frames

vid = VideoReader(vid_path);
numframes = floor(vid.Duration * vid.FrameRate);

num_segments = 12;
Segment_pts_x = zeros(numframes, num_segments+1);
Segment_pts_y = zeros(numframes, num_segments+1);
Angles = zeros(numframes, num_segments-1);
Angle0 = zeros(numframes, 1);

for i = 1:numframes
    % Read frame and analyse it
    im = readFrame(vid);
    im = mean(im, 3);
    segment_pts = segmentTracking(im, 'num_segments', num_segments, 'inertia', 0.2, 'body_length', 0, 'tail_length', 120, 'initial_box', 0.5,  'box_increment', 0.04);
    % If tail leaves screen, there is a nan value, let's correct it
%     if sum(isnan(segment_pts(:, 1))) ~= 0
%         for ind = find(isnan(segment_pts(:, 1)))
%             segment_pts(ind, 1) = 2 * Segment_pts_x(i-1, ind)' - Segment_pts_x(i-2, ind)';
%             segment_pts(ind, 2) = 2 * Segment_pts_y(i-1, ind)' - Segment_pts_y(i-2, ind)';
%         end
%     end
    % Save positions
    Segment_pts_x(i, :) = segment_pts(:, 1)';
    Segment_pts_y(i, :) = segment_pts(:, 2)';
    % Get angles
    [angles, angle0] = getTrackingAngles(segment_pts);
    Angles(i, :) = angles';
    Angle0(i) = angle0;
end


%% Plot segment angles

figure
subplot(num_segments, 1, 1)
plot(Angle0)
title('All segment angles from former segment', 'Interpreter', 'latex')
for i = 1:(num_segments-1)
    subplot(num_segments, 1, i+1)
    plot(Angles(:, i))
end


%% Plot total tail angle and differential

total_angle = sum([Angle0, Angles], 2);
figure
subplot(2, 1, 1)
plot(total_angle)
title('Total tail angle', 'Interpreter', 'latex')
subplot(2, 1, 2)
plot(diff(total_angle))
title('Differential of total tail angle', 'Interpreter', 'latex')


%% Plot segment positions

figure
subplot(num_segments+1, 2, 1)
plot(Segment_pts_x(:, 1))
title('x-axis position for all segment endpoints', 'Interpreter', 'latex')
subplot(num_segments+1, 2, 2)
plot(Segment_pts_y(:, 1))
title('y-axis position for all segment endpoints', 'Interpreter', 'latex')
for i = 1:num_segments
    subplot(num_segments+1, 2, 2*i+1)
    plot(Segment_pts_x(:, i+1))
    subplot(num_segments+1, 2, 2*(i+1))
    plot(Segment_pts_y(:, i+1))
end


%% Optional: create a tracking video

% Video creation
vid_path = '/home/ljp/Desktop/video.avi';

vid = VideoReader(vid_path);
numframes = floor(vid.Duration * vid.FrameRate);

writerObj = VideoWriter('/home/ljp/Desktop/baseline_19_1_10.avi');
writerObj.FrameRate = vid.FrameRate;
open(writerObj);

parameters.num_segments = 12;
parameters.inertia = 0.5;
parameters.body_length = 0;
parameters.tail_length = 115;
parameters.initial_box = 0.5;
parameters.box_increment = 0.04;

figure
% Plot evolution and get info
for i = 1:numframes
    % Read frame and analyse it
    im = readFrame(vid);
    im = mean(im, 3);
    [segment_pts, coms, polygons] = segmentTracking(im, 'num_segments', parameters.num_segments, 'inertia', parameters.inertia, 'body_length', parameters.body_length, ...
                                      'tail_length', parameters.tail_length, 'initial_box', parameters.initial_box,  'box_increment', parameters.box_increment);
    % Plot
    hold on
    image(im, 'CDataMapping', 'scaled')
    plot(coms(:, 2), coms(:, 1), 'o')
    plot(segment_pts(:, 2), segment_pts(:, 1), 'r')
    for j = 1:length(polygons)
        plot(polygons{j}(:, 2), polygons{j}(:, 1), 'g')
    end
    axis equal
    hold off
    % Get frame and save in video
    F = getframe;
    if i == 1
        xlen = size(F.cdata, 1);
        ylen = size(F.cdata, 2);
    else
        F.cdata = imresize(F.cdata, [xlen, ylen]);
    end
    writeVideo(writerObj, F);
end

close(writerObj)


%% boutWrapper test

tracking = boutsWrapper(vid_path, 'num_segments', parameters.num_segments, 'inertia', parameters.inertia, 'body_length', parameters.body_length, ...
                                      'tail_length', parameters.tail_length, 'initial_box', parameters.initial_box,  'box_increment', parameters.box_increment, ...
                                      'trigger_value', 15, 'num_pts_after', 5);
    




