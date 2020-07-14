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
            % Load obj one after the other and save sum(obj.TailBout)
            try
                % Create vector to save later
                bouts = zeros(1, 4);
                baseline = fullfile(run_path, run.name, '1-1-baseline.mat');
                load(baseline);
                bouts(1) = sum(obj.TailBout);
                vestibular = fullfile(run_path, run.name, '1-2-vestibular.mat');
                load(vestibular);
                bouts(2) = sum(obj.TailBout);
                vestibular = fullfile(run_path, run.name, '1-3-vestibular.mat');
                load(vestibular);
                bouts(3) = sum(obj.TailBout);
                vestibular = fullfile(run_path, run.name, '1-4-vestibular.mat');
                load(vestibular);
                bouts(4) = sum(obj.TailBout);
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
            catch
                try
                    % Create vector to save later
                    bouts = zeros(1, 3);
                    vestibular = fullfile(run_path, run.name, '1-1-vestibular.mat');
                    load(vestibular);
                    bouts(1) = sum(obj.TailBout);
                    vestibular = fullfile(run_path, run.name, '1-2-vestibular.mat');
                    load(vestibular);
                    bouts(2) = sum(obj.TailBout);
                    vestibular = fullfile(run_path, run.name, '1-3-vestibular.mat');
                    load(vestibular);
                    bouts(3) = sum(obj.TailBout);
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
                catch
                    continue
                end
            end
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

    



