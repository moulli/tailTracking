clear; close all; clc


sel_path1 = '/home/ljp/Science/balancoire_data/2020-02/19/fish1_7dpf/run10/1-1-baseline.avi';
sel_path2 = '/home/ljp/Science/balancoire_data/2020-02/20/fish2_6dpf/run1/1-2-vestibular.avi';
sel_path3 = '/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run1/1-3-vestibular.avi';
sel_path4 = '/home/ljp/Science/balancoire_data/2020-02/20/fish1_6dpf/run3/1-1-baseline.avi';
sel_path5 = '/home/ljp/Science/balancoire_data/2020-02/12/fish2_7dpf/run2/1-3-vestibular.avi';
sel_path6 = '/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run1/1-2-vestibular.avi';
sel_path7 = '/home/ljp/Science/balancoire_data/2020-02/12/fish2_7dpf/run9/1-4-vestibular.avi';
sel_path8 = '/home/ljp/Science/balancoire_data/2020-02/13/fish3_8dpf/run2/1-3-vestibular.avi';
sel_path9 = '/home/ljp/Science/balancoire_data/2020-02/19/fish3_7dpf/run1/1-2-vestibular.avi';
sel_path10 = '/home/ljp/Science/balancoire_data/2020-02/18/fish1_6dpf/run5/1-2-vestibular.avi';
sel_path11 = '/home/ljp/Desktop/video.avi';

selected_path = sel_path11;


%% Launch analysis + visual

vid = VideoReader(selected_path);
init = 1;
numframes = 50; %floor(vid.Duration * vid.FrameRate);
% Skip first frames
for f = 1:(init-1)
    im = readFrame(vid);
end
% Define vectors
num_segments = 9;
Angles = zeros(numframes-init+1, num_segments-1);
Angle0 = zeros(numframes-init+1, 1);
Pt0 = zeros(numframes-init+1, 2);
Segment_pts_x = zeros(numframes-init+1, num_segments+1);
Segment_pts_y = zeros(numframes-init+1, num_segments+1);
% Plot evolution and get info
figure
for i = init:numframes
    % Read frame and analyse it
    im = readFrame(vid);
    im = mean(im, 3);
    [segment_pts, coms, polygons] = segmentTracking(im, 'flip', 4, 'num_segments', num_segments, 'tail_length', 110, 'body_length', 0, 'inertia', 0.5, 'num_pixels_pca', 1500);
    % If tail leaves screen, there is a nan value, let's correct it
%     if sum(isnan(segment_pts(:, 1))) ~= 0
%         for ind = find(isnan(segment_pts(:, 1)))
%             segment_pts(ind, 1) = 2 * Segment_pts_x(i-init, ind)' - Segment_pts_x(i-init-1, ind)';
%             segment_pts(ind, 2) = 2 * Segment_pts_y(i-init, ind)' - Segment_pts_y(i-init-1, ind)';
%         end
%     end
    % Save positions
    Segment_pts_x(i-init+1, :) = segment_pts(:, 1)';
    Segment_pts_y(i-init+1, :) = segment_pts(:, 2)';
    % Get angles
    [angles, angle0, pt0] = getTrackingAngles(segment_pts);
    Angles(i-init+1, :) = angles';
    Angle0(i-init+1) = angle0;
    Pt0(i-init+1, :) = pt0;
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
    title(i)
    pause(0.1)
    drawnow limitrate
end
drawnow


%% Plot data

fixed_part = 0.75;
absolute_x = Segment_pts_x(:, 2:end) - (fixed_part*Segment_pts_x(:, 1) + (1-fixed_part)*mean(Segment_pts_x, 2));
absoluten_x = (absolute_x - mean(absolute_x)) ./ (std(absolute_x) + (std(absolute_x) == 0));
absend_x = absoluten_x(:, end);
absolute_y = Segment_pts_y(:, 2:end) - (fixed_part*Segment_pts_y(:, 1) + (1-fixed_part)*mean(Segment_pts_y, 2));
absoluten_y = (absolute_y - mean(absolute_y)) ./ (std(absolute_y) + (std(absolute_y) == 0));
absend_y = absoluten_y(:, end);
abs_absoluten = sqrt(absoluten_x.^2 + absoluten_y.^2);

figure
subplot(4, 1, 1)
plot(absend_x, '.:')
title('Absolute displacement along x-axis of end of fish', 'Interpreter', 'latex')
subplot(4, 1, 2)
plot(absend_y, '.:')
title('Absolute displacement along y-axis of end of fish', 'Interpreter', 'latex')
subplot(4, 1, 3)
hold on
plot(abs_absoluten(:, end), '.:')
plot(5*(abs_absoluten(:, end) > 3 * std(abs_absoluten(:, end))))
title('Absolute displacement of end of fish', 'Interpreter', 'latex')
subplot(4, 1, 4)
hold on
int_absoluten_end = conv(abs_absoluten(:, end), ones(2, 1));
plot(abs_absoluten(:, end), '.:')
plot(5*(int_absoluten_end > 3 * std(abs_absoluten(:, end))))
title('Absolute displacement of end of fish', 'Interpreter', 'latex')

% figure
% indexes = 6:3:12;
% for i = 1:3
%     j = indexes(i);
%     subplot(length(indexes), 1, i)
%     plot(abs_absoluten(:, j), '.:')
%     axis([1 size(abs_absoluten, 1) -8 8])
%     if i == 1
%         title('Absolute displacement for each segment', 'Interpreter', 'latex')
%     end
% end
% move = sqrt(sum(abs_absoluten(:, indexes).^2, 2));
% figure
% plot(move)


%% Analyse angles

% parameters
trigger_value = 9;
after = 9;


figure

angle_tot = sum([Angle0, Angles], 2);
ax1 = subplot(5, 1, 1);
x1 = 1:length(angle_tot);
y1 = angle_tot;
plot(x1, y1, '.-')
title('Total angle of tail', 'Interpreter', 'latex')

ax2 = subplot(5, 1, 2);
diff_angle = diff(angle_tot);
x2 = 2:(length(diff_angle)+1);
y2 = diff_angle;
plot(x2, y2, '.-')
title('Differential of total tail angle', 'Interpreter', 'latex')

ax3 = subplot(5, 1, 3);
trigger = abs(diff_angle) > trigger_value;
x3 = 2:(length(trigger)+1);
y3 = trigger;
plot(x3, y3, 'r')
title('Bouts detected', 'Interpreter', 'latex')

ax4 = subplot(5, 1, 4);
last_trigger = 0;
index = 1;
integrate = zeros(size(angle_tot));
while index <= length(diff_angle)
    % Define reference angle if new trigger
    if trigger(index) && last_trigger == 0
        former_angle = angle_tot(index);
    end
    % Compute integration
    if trigger(index)
        integrate(index+1) = integrate(index) + abs(angle_tot(index+1)-former_angle);
        last_trigger = after;
    elseif last_trigger > 0
        integrate(index+1) = integrate(index) + abs(angle_tot(index+1)-former_angle);
        last_trigger = max([0, last_trigger-1]);
    end
    index = index + 1;
end
x4 = 1:length(integrate);
y4 = integrate;
plot(x4, y4, 'g')
title('Integration on movement for each bout', 'Interpreter', 'latex')

ax5 = subplot(5, 1, 5);
final_bouts = zeros(size(integrate));
for i = 2:length(final_bouts)
    if integrate(i-1) == 0 && integrate(i) ~= 0
        final_bouts(i) = 1;
    end
end
x5 = 1:length(final_bouts);
y5 = final_bouts;
plot(x5, y5, 'r')
title('Final repartition of bouts', 'Interpreter', 'latex')

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
    
figure
hold on
plot(diff(tail_elipse), ':')
plot(diff_angle)
legend('differential before', 'differential using tail tracking')
title('Comparison between the two tracking algorithms', 'Interpreter', 'latex')
xlabel('Time', 'Interpreter', 'latex')
    


%% Try another technique

fixed_part = 1;
absolute_x = Segment_pts_x(:, 2:end) - (fixed_part*Segment_pts_x(:, 1) + (1-fixed_part)*mean(Segment_pts_x, 2));
diff_abs_x = diff(absolute_x(:, end)); % .* (diff(absolute_x(:, end)) > 1 | diff(absolute_x(:, end)) < -1);
diff_abs_x = conv((diff_abs_x), ones(3, 1));
absolute_y = Segment_pts_y(:, 2:end) - (fixed_part*Segment_pts_y(:, 1) + (1-fixed_part)*mean(Segment_pts_y, 2));
diff_abs_y = diff(absolute_y(:, end)) .* (diff(absolute_y(:, end)) > 1 | diff(absolute_y(:, end)) < -1);


%% Try boutsWrapper

[bouts, bouts_intensity] = boutsWrapper(selected_path, 3);


%% Create video

% Video creation
writerObj = VideoWriter('newvid.avi');
writerObj.FrameRate = vid.FrameRate;
open(writerObj);

vid = VideoReader(selected_path);
init = 1;
numframes = floor(vid.Duration * vid.FrameRate);
% Define vectors
num_segments = 9;
figure
% Plot evolution and get info
for i = init:numframes
    % Read frame and analyse it
    im = readFrame(vid);
    [segment_pts, coms, polygons] = segmentTracking(im, 'flip', 3, 'num_segments', num_segments, 'tail_length', 110, 'body_length', 0, 'inertia', 0.5);
    % If tail leaves screen, there is a nan value, let's correct it
    if sum(isnan(segment_pts(:, 1))) ~= 0
        for ind = find(isnan(segment_pts(:, 1)))
            segment_pts(ind, 1) = 2 * Segment_pts_x(i-init, ind)' - Segment_pts_x(i-init-1, ind)';
            segment_pts(ind, 2) = 2 * Segment_pts_y(i-init, ind)' - Segment_pts_y(i-init-1, ind)';
        end
    end
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
    writeVideo(writerObj, F);
    
end

close(writerObj)







