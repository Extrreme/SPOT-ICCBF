%% Get idx

QV_startTime = dataClass.Time_s.Data(find(dataClass.RED_Control_Law_Enabler.Data == 3, 1, 'first'));
QV_endTime = dataClass.Time_s.Data(find(dataClass.RED_Control_Law_Enabler.Data == 3, 1, 'last'));

QV_idx = find(dataClass.Time_s.Data == QV_startTime):1:find(dataClass.Time_s.Data == QV_endTime);

%% Plot CBFs

QV_x_RED = [dataClass.RED_Px_m.Data(QV_idx,:), dataClass.RED_Py_m.Data(QV_idx,:), dataClass.RED_Rz_rad.Data(QV_idx,:)];
QV_x_BLACK = [dataClass.BLACK_Px_m.Data(QV_idx,:), dataClass.BLACK_Py_m.Data(QV_idx,:), dataClass.BLACK_Rz_rad.Data(QV_idx,:)];
QV_x_BLUE = [dataClass.BLUE_Px_m.Data(QV_idx,:), dataClass.BLUE_Py_m.Data(QV_idx,:), dataClass.BLUE_Rz_rad.Data(QV_idx,:)];

QV_r_KOZ_tar = dataClass.CBF_Target_KOZ_Radius.Data(QV_idx,:);

QV_h_LOS = zeros(length(dataClass.Time_s.Data(QV_idx,:)),1);
QV_h_KOZ_tar = zeros(length(dataClass.Time_s.Data(QV_idx,:)),1);
QV_h_KOZ_obs = zeros(length(dataClass.Time_s.Data(QV_idx,:)),1);

for j = 1:length(dataClass.Time_s.Data(QV_idx,:))
    QV_S_tar = diag(1./QV_r_KOZ_tar.^2);
    QV_R_tar = [cos(QV_x_BLACK(j,3)) -sin(QV_x_BLACK(j,3));
                sin(QV_x_BLACK(j,3))  cos(QV_x_BLACK(j,3))];

    QV_S_obs = diag(1./r_KOZ_obs.^2);
    QV_R_obs = [cos(QV_x_BLUE(j,3)) -sin(QV_x_BLUE(j,3));
                sin(QV_x_BLUE(j,3))  cos(QV_x_BLUE(j,3))];

    QV_R_chs = [cos(QV_x_RED(j,3)) -sin(QV_x_RED(j,3));
                sin(QV_x_RED(j,3))  cos(QV_x_RED(j,3))];
    QV_r_LOS = (QV_x_BLACK(j,1:2)'-QV_x_RED(j,1:2)'- QV_R_chs*sensor_offset);

    QV_h_LOS(j,:) = (QV_r_LOS'*(QV_R_chs*sensor_normal))^2 - QV_r_LOS'*QV_r_LOS*(cos(sensor_FOV))^2;
    QV_h_KOZ_tar(j,:) = (QV_x_RED(1:2) - QV_x_BLACK(1:2))'*QV_R_tar*QV_S_tar*QV_R_tar'*(QV_x_RED(1:2) - QV_x_BLACK(1:2)) - 1;
    QV_h_KOZ_obs(j,:) = (QV_x_RED(1:2) - QV_x_BLUE(1:2))'*QV_R_obs*QV_S_obs*QV_R_obs'*(QV_x_RED(1:2) - QV_x_BLUE(1:2)) - 1;
end


figure
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
hold on

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), h_KOZ_tar, 'Color', 'k', 'LineWidth', 1.05);
plot(dataClass.Time_s.Data(QV_idx(h_KOZ_tar < 0),:) - dataClass.Time_s.Data(QV_idx(1),:), h_KOZ_tar(h_KOZ_tar < 0), 'Color', 'r', 'LineWidth', 1.05)

grid
ylabel("h_{KOZ,tar}")
ylim([min(h_KOZ_tar(:)) max(h_KOZ_tar(:))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), h_KOZ_obs, 'Color', 'k', 'LineWidth', 1.05);
plot(dataClass.Time_s.Data(QV_idx(h_KOZ_obs < 0),:) - dataClass.Time_s.Data(QV_idx(1),:), h_KOZ_obs(h_KOZ_obs < 0), 'Color', 'r', 'LineWidth', 1.05)

grid on
ylabel("h_{KOZ,obs}")
ylim([min(h_KOZ_obs(:)) max(h_KOZ_obs(:))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), h_LOS, 'Color', 'k', 'LineWidth', 1.05);
plot(dataClass.Time_s.Data(QV_idx(h_LOS < 0),:) - dataClass.Time_s.Data(QV_idx(1),:), h_LOS(h_LOS < 0), 'Color', 'r', 'LineWidth', 1.05)

grid
xlabel("Time [s]")
ylabel("h_{LOS}")
ylim([min(h_LOS(:)) max(h_LOS(:))])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

%% Plot Forces

figure
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
hold on

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), dataClass.CBF_Control.Data(QV_idx,1), 'Color', 'k', 'LineWidth', 1.05)

legend
grid
ylabel("F_x [N]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), dataClass.CBF_Control.Data(QV_idx,2), 'Color', 'k', 'LineWidth', 1.05)

grid
ylabel("F_y [N]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

nexttile
hold on

plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), dataClass.CBF_Control.Data(QV_idx,3), 'Color', 'k', 'LineWidth', 1.05)

grid
xlabel("Time [s]")
ylabel("T_z [Nm]")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

%% Plot Convergence

figure
hold on
plot(dataClass.Time_s.Data(QV_idx,:) - dataClass.Time_s.Data(QV_idx(1),:), dataClass.CBF_QP_Solver_Result.Data(QV_idx,1))
grid
xlabel("Time [s]")
ylabel("Convergence")
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12)

%% Clear Variables

clear QV_startTime QV_endTime QV_idx QV_x_RED QV_x_BLACK QV_x_BLUE ...
    QV_r_KOZ_tar QV_h_LOS QV_h_KOZ_tar QV_h_KOZ_obs QV_S_tar QV_R_tar ...
    QV_S_obs QV_R_obs QV_R_chs QV_r_LOS