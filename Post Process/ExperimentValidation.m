clc
clear all
close all

% Adding parent directory to the path, which contains the plotting functions
parentDirectory = fileparts(cd);
addpath(parentDirectory)          
addpath("functions")

%% Constants

yLength = 2.4;
xLength = 3.5;

baseRate = 0.05;

%% Select Data File

[MAT_FILES, MAT_DIR] = uigetfile('*', 'Select the MAT file', '../Saved Data/', 'MultiSelect', 'on');
MAT_FILEPATHS = fullfile(MAT_DIR, MAT_FILES);

%% Load Data

exp_trials = {};
sim_trials = {};

exp_idx = {};
sim_idx = {};

for j = 1:numel(MAT_FILEPATHS)
   [trials_j, idx_j, exp_j] = loadData(MAT_FILEPATHS{j});

   if exp_j
       exp_trials{end+1} = trials_j;
       exp_idx{end+1} = idx_j;
   else
       sim_trials{end+1} = trials_j;
       sim_idx{end+1} = idx_j;
   end
end

minSize = min(cellfun(@numel, [exp_idx sim_idx]));

%% Compute experiment trials mean and variance

RED_Px_mean = zeros(minSize,1);
RED_Py_mean = zeros(minSize,1);
RED_Rz_mean = zeros(minSize,1);

RED_Px_var = zeros(minSize,1);
RED_Py_var = zeros(minSize,1);
RED_Rz_var = zeros(minSize,1);

for i = 1:minSize
    RED_Px_sum = 0;
    RED_Py_sum = 0;
    RED_Rz_sum = 0;

    RED_Px_ssd = 0;
    RED_Py_ssd = 0;
    RED_Rz_ssd = 0;

    N = 0;
    for j = 1:numel(exp_trials)
        if i <= length(exp_idx{j})
            RED_Px_sum = RED_Px_sum + exp_trials{j}.RED_Px(exp_idx{j}(i));
            RED_Py_sum = RED_Py_sum + exp_trials{j}.RED_Py(exp_idx{j}(i));
            RED_Rz_sum = RED_Rz_sum + exp_trials{j}.RED_Rz(exp_idx{j}(i));

            N = N + 1;
        end
    end

    RED_Px_mean(i) = RED_Px_sum/N;
    RED_Py_mean(i) = RED_Py_sum/N;
    RED_Rz_mean(i) = RED_Rz_sum/N;

    for j = 1:numel(exp_trials)
        if i <= length(exp_idx{j})
            RED_Px_ssd = RED_Px_ssd + (exp_trials{j}.RED_Px(exp_idx{j}(i)) - RED_Px_mean(i))^2;
            RED_Py_ssd = RED_Py_ssd + (exp_trials{j}.RED_Py(exp_idx{j}(i)) - RED_Py_mean(i))^2;
            RED_Rz_ssd = RED_Rz_ssd + (exp_trials{j}.RED_Rz(exp_idx{j}(i)) - RED_Rz_mean(i))^2;
        end
    end

    RED_Px_var(i) = RED_Px_ssd/N;
    RED_Py_var(i) = RED_Py_ssd/N;
    RED_Rz_var(i) = RED_Rz_ssd/N;
end

%% Create Path Plot & Trial data

figure
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

snaps = [0 1/3 2/3 1];
snapsOpacity = [0.05 0.1 0.25 1];

nexttile([3, 1])
hold on
for j = 1:numel(sim_trials)
    idx_j = sim_idx{j}(1:minSize);

    RED_Px = sim_trials{j}.RED_Px;
    RED_Py = sim_trials{j}.RED_Py;
    RED_Rz = sim_trials{j}.RED_Rz;
    BLACK_Px = sim_trials{j}.BLACK_Px;
    BLACK_Py = sim_trials{j}.BLACK_Py;
    BLACK_Rz = sim_trials{j}.BLACK_Rz;

    patchline(RED_Px(idx_j), RED_Py(idx_j), 'k', 'FaceAlpha', 0, 'LineWidth', 0.6, 'EdgeColor', 'black', 'LineStyle', '--', 'HandleVisibility', 'off');
end

for j = 1:numel(exp_trials)
    idx_j = exp_idx{j}(1:minSize);

    RED_Px = exp_trials{j}.RED_Px;
    RED_Py = exp_trials{j}.RED_Py;
    RED_Rz = exp_trials{j}.RED_Rz;
    BLACK_Px = exp_trials{j}.BLACK_Px;
    BLACK_Py = exp_trials{j}.BLACK_Py;
    BLACK_Rz = exp_trials{j}.BLACK_Rz;

    patchline(RED_Px(idx_j), RED_Py(idx_j), 'r', 'FaceAlpha', 0, 'LineWidth', 0.6, 'EdgeColor', 'red', 'HandleVisibility', 'off');
    patchline(BLACK_Px(idx_j), BLACK_Py(idx_j), 'k', 'FaceAlpha', 0, 'LineWidth', 0.6, 'EdgeColor', 'black', 'HandleVisibility', 'off')

    for k = 1:length(snaps)
        if snaps(k) == 0
            frame = 1;
        else
            frame = round(snaps(k)*length(idx_j));
        end

        spacecraft = DrawSpacecraft([RED_Px(frame), RED_Py(frame), RED_Rz(frame), 6]);
        patch(spacecraft(:,1), spacecraft(:,2), 'r', 'FaceAlpha', 0, 'LineWidth', 0.6, 'EdgeColor', 'red', 'EdgeAlpha', snapsOpacity(k), 'HandleVisibility', 'off')

        spacecraft = DrawSpacecraft([BLACK_Px(frame), BLACK_Py(frame), BLACK_Rz(frame), 4]);
        patch(spacecraft(:,1), spacecraft(:,2), 'k', 'FaceAlpha', 0, 'LineWidth', 0.6, 'EdgeColor', 'black', 'EdgeAlpha', snapsOpacity(k), 'HandleVisibility', 'off')
    end
end

if numel(sim_trials) > 0
    plot(nan, nan, 'k--', 'LineWidth', 0.6, 'DisplayName', 'Simulation');
    plot(nan, nan, 'r', 'LineWidth', 0.6, 'DisplayName', 'Experiment Trial');
end

grid on
box on
axis equal;

xlim([0 xLength]);
ylim([0 yLength]);

xlabel('X-Position [m]')
ylabel('Y-Position [m]')
legend('Location', 'northwest')

set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on
plot((0:baseRate:(minSize-1)*baseRate)', RED_Px_mean, 'k', 'DisplayName', 'Mean');
fill([(0:baseRate:(minSize-1)*baseRate)'; fliplr((0:baseRate:(minSize-1)*baseRate))'], ...
     [RED_Px_mean+3*sqrt(RED_Px_var); fliplr((RED_Px_mean-3*sqrt(RED_Px_var))')'], ...
     'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'LineStyle', '--', 'DisplayName', '3\sigma Limits');
grid on
box on
ylabel('{\it x}_s [m]')
xlim([0 (minSize-1)*baseRate])
ylim([0 3.5])
legend('Location', 'northoutside', 'NumColumns', 2)
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on
plot((0:baseRate:(minSize-1)*baseRate)', RED_Py_mean, 'k')
fill([(0:baseRate:(minSize-1)*baseRate)'; fliplr((0:baseRate:(minSize-1)*baseRate))'], ...
     [RED_Py_mean+3*sqrt(RED_Py_var); fliplr((RED_Py_mean-3*sqrt(RED_Py_var))')'], ...
     'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'LineStyle', '--', 'DisplayName', '3\sigma Limits');
grid on
box on
ylabel('{\it y}_s [m]')
xlim([0 (minSize-1)*baseRate])
ylim([0 2.5])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on
plot((0:baseRate:(minSize-1)*baseRate)', RED_Rz_mean, 'k')
fill([(0:baseRate:(minSize-1)*baseRate)'; fliplr((0:baseRate:(minSize-1)*baseRate))'], ...
     [RED_Rz_mean+3*sqrt(RED_Rz_var); fliplr((RED_Rz_mean-3*sqrt(RED_Rz_var))')'], ...
     'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'LineStyle', '--', 'DisplayName', '3\sigma Limits');
grid on
box on
ylabel('\theta_s [rad]')
xlabel('Time [s]')
xlim([0 (minSize-1)*baseRate])
ylim([-3 1])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

set(gcf, 'Position', [gcf().Position(1:2), 1160, 420]);
set(gcf, 'PaperOrientation', 'landscape');
set(gcf, 'PaperSize', [18 8.5]);

%% Functions

function [data, idx, exp] = loadData(mat_file)
    dataClass = load(mat_file);
    if isfield(dataClass, "dataClass")
        dataClass = dataClass.dataClass;
        exp = 0;
    else
        dataClass = dataClass.dataClass_rt;
        exp = 1;
    end

    data = struct();

    startIdx = find(dataClass.RED_Control_Law_Enabler.Data == 3);
    idx = startIdx:1:length(dataClass.Time_s.Data);

    data.time = dataClass.Time_s.Data(idx,:);
    
    data.RED_Px = dataClass.RED_Px_m.Data(idx,:);
    data.RED_Py = dataClass.RED_Py_m.Data(idx,:);
    data.RED_Rz = dataClass.RED_Rz_rad.Data(idx,:);
    
    data.BLACK_Px = dataClass.BLACK_Px_m.Data(idx,:);
    data.BLACK_Py = dataClass.BLACK_Py_m.Data(idx,:);
    data.BLACK_Rz = dataClass.BLACK_Rz_rad.Data(idx,:);
    
    data.RED_Fx = dataClass.RED_Fx_N.Data(idx,:);
    data.RED_Fy = dataClass.RED_Fy_N.Data(idx,:);
    data.RED_Fz = dataClass.RED_Tz_Nm.Data(idx,:);
    
    data.BLACK_Fx = dataClass.BLACK_Fx_N.Data(idx,:);
    data.BLACK_Fy = dataClass.BLACK_Fy_N.Data(idx,:);
    data.BLACK_Tz = dataClass.BLACK_Tz_Nm.Data(idx,:);

    data.KOZ = dataClass.Target_KOZ.Data(idx,:);

    idx = 1:length(data.time);
end