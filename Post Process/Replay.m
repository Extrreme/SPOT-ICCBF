clc
clear all
close all

% Adding parent directory to the path, which contains the plotting functions
parentDirectory = fileparts(cd);
addpath(parentDirectory)          
addpath("../functions")

%% Base Parameters
d2r = pi/180;

yLength = 2.4;
xLength = 3.5;

baseRate = 0.05;

% Target Line-of-Sight
sensor_FOV = 40*d2r;                            % Sensor FOV [rad]
sensor_offset = [0.145-0.042 -0.0395]';         % Sensor body-fixed offset
sensor_normal = [1 0]';                         % Sensor normal vector
sensor_target = [0.0825 0.2516]';               % Desired pointing location (targed body-fixed) [m,m]

% Obstacle Keep-out-Zone
r_KOZ_obs = 0.43*[1 1];

%% Select Data File

[MAT_FILES, MAT_DIR] = uigetfile('*', 'Select the MAT file', '../Saved Data/', 'MultiSelect', 'on');

if isequal(MAT_FILES, 0)
    error("No files selected")
elseif ischar(MAT_FILES)
    MAT_FILES = {MAT_FILES};
end

MAT_FILEPATHS = fullfile(MAT_DIR, MAT_FILES);
names = {};
lineStyles = {"-", "--", "-."};
for k = 1:numel(MAT_FILEPATHS)
    if contains(MAT_FILEPATHS{k}, "TVICCBF")
        names{k} = "TVICCBF";
    elseif contains(MAT_FILEPATHS{k}, "TVCBF")
        names{k} = "TVCBF";
    elseif contains(MAT_FILEPATHS{k}, "CBF")
        names{k} = "CBF";
    end
end

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

%% Replay

trials = [exp_trials sim_trials];

fig = figure('Color', 'w');

frameStep = 3;
counter = 1;
for frame = 1:frameStep:min(cellfun(@numel, [exp_idx sim_idx]))
    clf
    patch([0, xLength, xLength, 0], [0, 0, yLength, yLength], 'black', 'FaceAlpha', 0.08, 'HandleVisibility', 'off'); 
    hold on

    text(2,2, ["Time: ", sim_trials{1}.time(frame)-sim_trials{1}.time(1) ]);
    for j = 1:numel(trials)
        plot(0,0,'r', 'LineStyle', lineStyles{j}, 'DisplayName', names{j});

        % Plotting trajectory
        patchline(trials{j}.RED_Px(1:frame), trials{j}.RED_Py(1:frame), 'r', 'EdgeColor', 'red', 'Linewidth', 1.2, 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j});
        patchline(trials{j}.BLACK_Px(1:frame), trials{j}.BLACK_Py(1:frame), 'k', 'EdgeColor', 'black','Linewidth', 1.2, 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j});
        patchline(trials{j}.BLUE_Px(1:frame), trials{j}.BLUE_Py(1:frame), 'b', 'EdgeColor', 'blue','Linewidth', 1.2, 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j});

        % Plotting spacecraft shapes
        spacecraft = DrawSpacecraft([trials{j}.RED_Px(frame), trials{j}.RED_Py(frame), trials{j}.RED_Rz(frame),6]);
        patch(spacecraft(:,1), spacecraft(:,2), 'r', 'FaceAlpha', 0.2, 'LineWidth', 0.6, 'EdgeColor', 'red', 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j})
    
        spacecraft = DrawSpacecraft([trials{j}.BLACK_Px(frame), trials{j}.BLACK_Py(frame), trials{j}.BLACK_Rz(frame),4]);
        patch(spacecraft(:,1), spacecraft(:,2), 'k', 'FaceAlpha', 0.2, 'LineWidth', 0.6, 'EdgeColor', 'black', 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j})
        
        spacecraft = DrawSpacecraft([trials{j}.BLUE_Px(frame), trials{j}.BLUE_Py(frame), trials{j}.BLUE_Rz(frame),3]);
        patch(spacecraft(:,1), spacecraft(:,2), 'b', 'FaceAlpha', 0.2, 'LineWidth', 0.6, 'EdgeColor', 'blue', 'EdgeAlpha', 1, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j})
        
        % Plot custom drawings
        koz = DrawEllipse(trials{j}.BLACK_Px(frame), trials{j}.BLACK_Py(frame), trials{j}.KOZ(frame,1), trials{j}.KOZ(frame,2), trials{j}.BLACK_Rz(frame));
        patch(koz(:,1), koz(:,2), 'k', 'FaceAlpha', 0.05, 'LineWidth', 0.6, 'EdgeColor', 'black', 'EdgeAlpha', 0.5, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j})

        koz = DrawEllipse(trials{j}.BLUE_Px(frame), trials{j}.BLUE_Py(frame), r_KOZ_obs(1), r_KOZ_obs(2), trials{j}.BLACK_Rz(frame));
        patch(koz(:,1), koz(:,2), 'b', 'FaceAlpha', 0.05, 'LineWidth', 0.6, 'EdgeColor', 'blue', 'EdgeAlpha', 0.5, 'HandleVisibility', 'off', 'LineStyle', lineStyles{j})
    
        R = [cos(trials{j}.RED_Rz(frame)) -sin(trials{j}.RED_Rz(frame));
             sin(trials{j}.RED_Rz(frame))  cos(trials{j}.RED_Rz(frame))];
    
        r1 = [trials{j}.RED_Px(frame); trials{j}.RED_Py(frame)] + R*sensor_offset;
        r2 = r1 + R*sensor_normal;
    
        [xp, yp, xm, ym] = calculateLineEndpoints(r1(1), r1(2), r2(1), r2(2), sensor_FOV, 10);
    
        patch([r1(1), xp, xm, r1(1)], [r1(2), yp, ym, r1(2)], 'r', 'EdgeColor', 'r', 'EdgeAlpha', 0.1, 'FaceAlpha', 0.025, 'LineStyle', '--', 'HandleVisibility', 'off')
    end

    grid on
    box on
    axis equal;

    set(gca, 'FontName', 'Times New Roman', 'FontSize', 10)

    xlim([-0.2 xLength+0.2]);
    ylim([-0.2 yLength+0.2]);

    xlabel('X-Position [m]');
    ylabel('Y-Position [m]');

    legend

    getframe(gcf);
    counter = counter+1;
end

% close(gcf)
% 
% [file, path] = uiputfile('*.mp4', 'Save File As');
% 
% if isequal(file, 0) || isequal(path,0)
%     return
% end
% 
% myWriter = VideoWriter(strcat(path, '\', file), 'MPEG-4');
% myWriter.FrameRate = round(1/(baseRate*frameStep));
% open(myWriter);
% for i=1:length(movieVector)
%     writeVideo(myWriter, movieVector(i));
% end
% close(myWriter);


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

    data.BLUE_Px = dataClass.BLUE_Px_m.Data(idx,:);
    data.BLUE_Py = dataClass.BLUE_Py_m.Data(idx,:);
    data.BLUE_Rz = dataClass.BLUE_Rz_rad.Data(idx,:);
    
    data.RED_Fx = dataClass.RED_Fx_N.Data(idx,:);
    data.RED_Fy = dataClass.RED_Fy_N.Data(idx,:);
    data.RED_Fz = dataClass.RED_Tz_Nm.Data(idx,:);
    
    data.BLACK_Fx = dataClass.BLACK_Fx_N.Data(idx,:);
    data.BLACK_Fy = dataClass.BLACK_Fy_N.Data(idx,:);
    data.BLACK_Tz = dataClass.BLACK_Tz_Nm.Data(idx,:);

    data.KOZ = dataClass.Target_KOZ.Data(idx,:);

    idx = 1:length(data.time);
end

function [x_plus, y_plus, x_minus, y_minus] = calculateLineEndpoints(x1, y1, x2, y2, angle, lineLength)
    theta = atan2(y2 - y1, x2 - x1);

    x_plus = x2 + lineLength * cos(theta + angle);
    y_plus = y2 + lineLength * sin(theta + angle);

    x_minus = x2 + lineLength * cos(theta - angle);
    y_minus = y2 + lineLength * sin(theta - angle);
end