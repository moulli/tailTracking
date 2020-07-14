clear; close all; clc


vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/19/fish1_7dpf/run10/1-1-baseline.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/20/fish2_6dpf/run1/1-2-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run1/1-3-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/20/fish1_6dpf/run3/1-1-baseline.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run1/1-3-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/12/fish2_7dpf/run2/1-3-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/12/fish1_7dpf/run1/1-2-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/12/fish2_7dpf/run9/1-4-vestibular.avi');
vid = VideoReader('/home/ljp/Science/balancoire_data/2020-02/13/fish3_8dpf/run2/1-3-vestibular.avi');

selected_path = '/home/ljp/Science/balancoire_data/2020-02/13/fish3_8dpf/run2/1-3-vestibular.avi';
vid = VideoReader(selected_path);

% im = readFrame(vid);
im = read(vid, 490);
figure
imshow(im)

% Isolate pixels black enough to be the fish
figure
imshow(im < quantile(im(:), 0.02))
[fx, fy] = ind2sub(size(im), find(im < quantile(im(:), 0.02)));
figure
plot(fx, fy, '.')
axis equal
% % Find fish axis with linear regression
% fx = [ones(size(fx)), fx];
% angle0 = (fx' * fx) \ fx' * fy; % DOES NOT WORK WELL
% Find fish axis using PCA
pca0 = pca([fx, fy]);
vect0 = pca0(1, :);
vect0 = [-vect0(1), -vect0(2)]; % ATTENTION: EN FONCTION DE L'ORIENTATION, IL FAUT PARFOIS REMPLACER VECT0(2) PAR -VECT0(2)

% Find beginning of tail
[fxe, fye] = ind2sub(size(im), find(im < quantile(im(:), 0.005)));
mid_eyes = mean([fxe, fye]);
dist = sqrt((52-39)^2 + (-115+144)^2);
beg_tail = round(mid_eyes + dist*vect0);

% Plot results
figure
hold on
plot(fx, fy, '.')
plot([mid_eyes(1), beg_tail(1)], [mid_eyes(2), beg_tail(2)])
axis equal

% Now finding tail segments
num_segs = 12;
tail = 85; % between 83 and 95. sqrt((83-52)^2+(-38+115)^2);
seg_len = tail/num_segs;
seg_pts = zeros(num_segs+1, 2);
seg_pts(1, :) = beg_tail;
% Meshgrid for coordinates
[Xt, Yt] = meshgrid(1:size(im, 1), 1:size(im, 2));
Xt = Xt(:);
Yt = Yt(:);
coms = zeros(num_segs, 2);
polygons = cell(num_segs, 1);
for i = 1:num_segs
    % Define region of interest
    pt1 = seg_pts(i, :) + 0.3*seg_len*[vect0(2), -vect0(1)];
    pt2 = pt1 + seg_len*vect0;
    pt4 = seg_pts(i, :) + 0.3*seg_len*[-vect0(2), vect0(1)];
    pt3 = pt4 + seg_len*vect0;
    polydef = [pt1; pt2; pt3; pt4; pt1];
    polygons{i} = polydef;
    % Get points inside region
    pts_in_region = inpolygon(Xt(:), Yt(:), polydef(:, 1), polydef(:, 2));
    % Check position of rectangle
%     figure
%     plot(polydef(:, 1), polydef(:, 2)) % polygon
%     axis equal
%     hold on
%     plot(Xt(pts_in_region),Yt(pts_in_region),'r+') % points inside
%     plot(Xt(~pts_in_region),Yt(~pts_in_region),'bo') % points outside
%     hold off    
    pts_to_keep = sub2ind(size(im), Xt(pts_in_region), Yt(pts_in_region));
    im_vals = (256 - double(im(pts_to_keep))).^2;
    % Find center of mass, vector, and end of new segment
    com = [im_vals'*Xt(pts_in_region)/sum(im_vals), im_vals'*Yt(pts_in_region)/sum(im_vals)];
    coms(i, :) = com;
    vect = com - seg_pts(i, :);
    vect0 = vect ./ sqrt(sum(vect.^2));
    seg_pts(i+1, :) = round(seg_pts(i, :) + seg_len*vect0);
end

figure
hold on
image(im, 'CDataMapping', 'scaled')
plot(coms(:, 2), coms(:, 1), 'o')
plot(seg_pts(:, 2), seg_pts(:, 1), 'r')
for i = 1:num_segs
    plot(polygons{i}(:, 2), polygons{i}(:, 1), 'g')
end
axis equal


% Testing function
[segment_pts, coms, polygons] = segmentTracking(im, 'flipud', true, 'fliplr', true);
figure
hold on
image(im, 'CDataMapping', 'scaled')
plot(coms(:, 2), coms(:, 1), 'o')
plot(segment_pts(:, 2), segment_pts(:, 1), 'r')
for i = 1:length(polygons)
    plot(polygons{i}(:, 2), polygons{i}(:, 1), 'g')
end
axis equal

%% Test on movie
vid = VideoReader(selected_path);
init = 500;
numframes = 700;
figure
for i = init:numframes
    im = flipud(readFrame(vid)');
    [segment_pts, coms, polygons] = segmentTracking(im, 'flipud', true, 'fliplr', true);
    hold on
    image(im, 'CDataMapping', 'scaled')
    plot(coms(:, 2), coms(:, 1), 'o')
    plot(segment_pts(:, 2), segment_pts(:, 1), 'r')
    for j = 1:length(polygons)
        plot(polygons{j}(:, 2), polygons{j}(:, 1), 'g')
    end
    axis equal
    drawnow limitrate
    hold off
    pause(0.01)
end

%% Testing getTrackingAngles
vid = VideoReader(selected_path);
Angles = zeros(numframes-init+1, 11);
Angle0 = zeros(numframes-init+1, 1);
Pt0 = zeros(numframes-init+1, 2);
for i = init:numframes
    im = flipud(readFrame(vid)');
    segment_pts = segmentTracking(im, 'flipud', true, 'fliplr', true);
    [angles, angle0, pt0] = getTrackingAngles(segment_pts);
    Angles(i-init+1, :) = angles';
    Angle0(i-init+1) = angle0;
    Pt0(i-init+1, :) = pt0;
end
figure
subplot(2, 1, 1)
plot(Pt0(:, 1))
title('Position of point 0 projected on x-axis across trial', 'Interpreter', 'latex')
subplot(2, 1, 2)
plot(Pt0(:, 2))
title('Position of point 0 projected on y-axis across trial', 'Interpreter', 'latex')
figure
plot(Angle0)
title('Initial absolute angle in degress across trial', 'Interpreter', 'latex')
figure
Angles_sum = cumsum(Angles, 2);
for seg = 1:11
    subplot(11, 1, seg)
    hold on
    plot(Angles(:, seg))
    plot(Angles_sum(:, seg))
    if seg == 1
        title('All angles in degrees across trial (highest: closest to eyes, lowest: end of tail', 'Interpreter', 'latex')
    end
end
figure
Angles_diff = diff(Angles, 2);
Angles_diff_sum = cumsum(Angles_diff, 2);
for seg = 1:11
    subplot(11, 1, seg)
    hold on
    plot(Angles_diff(:, seg))
    plot(Angles_diff_sum(:, seg))
    if seg == 1
        title('All angle differences in degrees across trial (highest: closest to eyes, lowest: end of tail', 'Interpreter', 'latex')
    end
end

%% Try analysis with absolute position
vid = VideoReader(selected_path);
Segment_pts_x = zeros(numframes-init+1, 13);
Segment_pts_y = zeros(numframes-init+1, 13);
for i = init:numframes
    im = flipud(readFrame(vid)');
    segment_pts = segmentTracking(im, 'flipud', true, 'fliplr', true);
    Segment_pts_x(i-init+1, :) = segment_pts(:, 1)';
    Segment_pts_y(i-init+1, :) = segment_pts(:, 2)';
end
absolute_x = Segment_pts_x(:, 2:end) - Segment_pts_x(:, 1);
absolute_y = Segment_pts_y(:, 2:end) - Segment_pts_y(:, 1);
figure
for seg = 1:12
    subplot(12, 2, 2*seg-1)
    plot(absolute_x(:, seg))
    if seg == 1
        title('Absolute displacement along x-axis of end of segments', 'Interpreter', 'latex')
    end
    subplot(12, 2, 2*seg)
    plot(absolute_y(:, seg))
    if seg == 1
        title('Absolute displacement along y-axis of end of segments', 'Interpreter', 'latex')
    end
end
figure
subplot(2, 1, 1)
plot(absolute_x(:, end))
title('Absolute displacement along x-axis of end of fish', 'Interpreter', 'latex')
subplot(2, 1, 2)
plot(absolute_y(:, end))
title('Absolute displacement along y-axis of end of fish', 'Interpreter', 'latex')
    