clc
clear all
close all

addpath("../other/")
addpath("../CBF/")

%% Params
d2r = pi/180;

sensor_FOV = 35*d2r;                            % Sensor FOV [rad]
sensor_offset = [0.15 -0.0825]';         % Sensor body-fixed offset
sensor_normal = [1 0]';                         % Sensor normal vector
sensor_target = [0.0825 0.2516]';               % Desired pointing location (targed body-fixed) [m,m]

r_KOZ_obs = 0.45*[1 1]';
 
%% Select Data File

[MAT_FILES, MAT_DIR] = uigetfile('*', 'Select the MAT file', '../Saved Data/', 'MultiSelect', 'on');

if isequal(MAT_FILES, 0)
    error("No files selected")
elseif ischar(MAT_FILES)
    MAT_FILES = {MAT_FILES};
end

MAT_FILEPATHS = fullfile(MAT_DIR, MAT_FILES);

names = {};
for j = 1:numel(MAT_FILEPATHS)
   [dataClass{j}, idx{j}, ~] = loadData(MAT_FILEPATHS{j});
   [~, names{j}, ~] = fileparts(MAT_FILES{j});
end

colors = {"r", "k", "b", "r", "k", "b", "r", "k", "b"};
lineStyles = {"-", "-", "-", "--", "--", "--", ":", ":", ":"};

%% Compute Constraints

for k = 1:numel(MAT_FILEPATHS)
    x_RED = [dataClass{k}.RED_Px, dataClass{k}.RED_Py, dataClass{k}.RED_Rz];
    x_BLACK = [dataClass{k}.BLACK_Px, dataClass{k}.BLACK_Py, dataClass{k}.BLACK_Rz];
    x_BLUE = [dataClass{k}.BLUE_Px, dataClass{k}.BLUE_Py, dataClass{k}.BLUE_Rz];

    r_KOZ_tar = dataClass{k}.KOZ;

    for j = 1:length(dataClass{k}.Time)
        A_tar = ((cos(x_BLACK(j,3))/r_KOZ_tar(j,1))^2 + (sin(x_BLACK(j,3))/r_KOZ_tar(j,2))^2);
        B_tar = ((sin(x_BLACK(j,3))/r_KOZ_tar(j,1))^2 + (cos(x_BLACK(j,3))/r_KOZ_tar(j,2))^2);
        C_tar = 2*sin(x_BLACK(j,3))*cos(x_BLACK(j,3))*(1/r_KOZ_tar(j,1)^2-1/r_KOZ_tar(j,2)^2);
    
        A_obs = ((cos(x_BLUE(j,3))/r_KOZ_obs(1))^2 + (sin(x_BLUE(j,3))/r_KOZ_obs(2))^2);
        B_obs = ((sin(x_BLUE(j,3))/r_KOZ_obs(1))^2 + (cos(x_BLUE(j,3))/r_KOZ_obs(2))^2);
        C_obs = 2*sin(x_BLUE(j,3))*cos(x_BLUE(j,3))*(1/r_KOZ_obs(1)^2-1/r_KOZ_obs(2)^2);
    
        R = [cos(x_RED(j,3)) -sin(x_RED(j,3));
             sin(x_RED(j,3))  cos(x_RED(j,3))];
        r = (x_BLACK(j,1:2)'-x_RED(j,1:2)'-R*sensor_offset);
    
        h_LOS{k}(j,:) = (r'*(R*sensor_normal))^2-r'*r*(cos(sensor_FOV))^2;
        h_KOZ_tar{k}(j,:) = A_tar*(x_RED(j,1)-x_BLACK(j,1))^2 + B_tar*(x_RED(j,2)-x_BLACK(j,2))^2 + C_tar*(x_RED(j,1)-x_BLACK(j,1))*(x_RED(j,2)-x_BLACK(j,2))-1;
        h_KOZ_obs{k}(j,:) = A_obs*(x_RED(j,1)-x_BLUE(j,1))^2 + B_obs*(x_RED(j,2)-x_BLUE(j,2))^2 + C_obs*(x_RED(j,1)-x_BLUE(j,1))*(x_RED(j,2)-x_BLUE(j,2))-1;
    end
    
    for j = 1:length(u_RED{k})
        F_RED_tot{k}(j,:) = sum(abs(dataClass{k}.RED_Fth(j,:)));
    end

    L_RED{k} = trapz(dataClass{k}.Time, F_RED_tot{k});

    h_KOZ_tar_min{k} = min(h_KOZ_tar{k});
    h_KOZ_obs_min{k} = min(h_KOZ_obs{k});
    h_LOS_min{k} = min(h_LOS{k});

end

%% Check and Plot Constraints

figure
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
hold on

for j = 1:numel(dataClass)
    plot(dataClass{j}.Time - dataClass{j}.Time(1), h_KOZ_tar{j}, 'Color', colors{j}, 'LineWidth', 1.05, 'LineStyle', lineStyles{j}, 'DisplayName', names{j});
end

grid
legend
ylabel("h_{KOZ,tar}")
ylim([min(cell2mat(h_KOZ_tar(:))) max(cell2mat(h_KOZ_tar(:)))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on

for j = 1:numel(dataClass)
    plot(dataClass{j}.Time - dataClass{j}.Time(1), h_KOZ_obs{j}, 'Color', colors{j}, 'LineWidth', 1.05, 'LineStyle', lineStyles{j}, 'DisplayName', names{j});
end

grid
ylabel("h_{KOZ,obs}")
ylim([min(cell2mat(h_KOZ_obs(:))) max(cell2mat(h_KOZ_obs(:)))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on

for j = 1:numel(dataClass)
    plot(dataClass{j}.Time - dataClass{j}.Time(1), h_LOS{j}, 'Color', colors{j}, 'LineWidth', 1.05, 'LineStyle', lineStyles{j}, 'DisplayName', names{j});
end

grid
xlabel("Time [s]")
ylabel("h_{LOS}")
ylim([min(cell2mat(h_LOS(:))) max(cell2mat(h_LOS(:)))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

%% Plot Forces

figure
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
hold on
for k = 1:numel(dataClass)
    plot(dataClass{k}.Time - dataClass{k}.Time(1), dataClass{k}.RED_Fx, 'Color', colors{k}, 'LineWidth', 1.05, 'LineStyle', lineStyles{k}, 'DisplayName', names{k})
end
legend
grid
ylabel("F_x [N]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on
for k = 1:numel(dataClass)
    plot(dataClass{k}.Time - dataClass{k}.Time(1), dataClass{k}.RED_Fy, 'Color', colors{k}, 'LineWidth', 1.05, 'LineStyle', lineStyles{k}, 'DisplayName', names{k})
end
grid
ylabel("F_y [N]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on
for k = 1:numel(dataClass)
    plot(dataClass{k}.Time - dataClass{k}.Time(1), dataClass{k}.RED_Tz, 'Color', colors{k}, 'LineWidth', 1.05, 'LineStyle', lineStyles{k}, 'DisplayName', names{k})
end
grid
xlabel("Time [s]")
ylabel("T_z [Nm]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

%% Plot Convergence

figure
hold on

for k = 1:numel(dataClass)
    plot(dataClass{k}.CBF_res(:,1), 'Color', colors{k}, 'LineWidth', 1.05, 'LineStyle', lineStyles{k}, 'DisplayName', names{k})
end

grid
xlabel("Time [s]")
ylabel("Convergence")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)
legend

%% Print Impulses

for k = 1:numel(dataClass)
    fprintf(names{k} + " - Fuel: " + L_RED{k} + " Ns\n")
end

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

    startTime = dataClass.Time_s.Data(find(dataClass.RED_Control_Law_Enabler.Data == 3, 1, 'first'));
    endTime = dataClass.Time_s.Data(find(dataClass.RED_Control_Law_Enabler.Data == 3, 1, 'last'));

    idx = find(dataClass.Time_s.Data == startTime):1:find(dataClass.Time_s.Data == endTime);

    data.Time = dataClass.Time_s.Data(idx,:);
    
    data.RED_Px = dataClass.RED_Px_m.Data(idx,:);
    data.RED_Py = dataClass.RED_Py_m.Data(idx,:);
    data.RED_Rz = dataClass.RED_Rz_rad.Data(idx,:);
    data.RED_Fth = dataClass.RED_Thruster_Controls_N.Data(idx,:);
    data.RED_Fth2 = dataClass.RED_Saturated_Thruster_Controls_N.Data(idx,:);

    data.BLACK_Px = dataClass.BLACK_Px_m.Data(idx,:);
    data.BLACK_Py = dataClass.BLACK_Py_m.Data(idx,:);
    data.BLACK_Rz = dataClass.BLACK_Rz_rad.Data(idx,:);

    data.BLUE_Px = dataClass.BLUE_Px_m.Data(idx,:);
    data.BLUE_Py = dataClass.BLUE_Py_m.Data(idx,:);
    data.BLUE_Rz = dataClass.BLUE_Rz_rad.Data(idx,:);
    
    data.RED_Fx = dataClass.RED_Fx_N.Data(idx,:);
    data.RED_Fy = dataClass.RED_Fy_N.Data(idx,:);
    data.RED_Tz = dataClass.RED_Tz_Nm.Data(idx,:);
    
    data.BLACK_Fx = dataClass.BLACK_Fx_N.Data(idx,:);
    data.BLACK_Fy = dataClass.BLACK_Fy_N.Data(idx,:);
    data.BLACK_Tz = dataClass.BLACK_Tz_Nm.Data(idx,:);

    data.KOZ = dataClass.CBF_Target_KOZ_Radius.Data(idx,:);

    data.CBF_res = dataClass.CBF_QP_Solver_Result.Data(idx,:);
    idx = 1:length(data.Time);
end