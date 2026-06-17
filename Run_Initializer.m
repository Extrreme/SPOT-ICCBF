% The following script is the initializer for SPOT 4.1; in this script,
% users define all initials parameters and/or constants required for
% simulation and experiment.

clear;
clc;
close all force;

warning('off','all')

parentDirectory = fileparts(cd);
addpath(parentDirectory)          
addpath("functions")
addpath("Post Process")

%% Start the graphical user interface or set the appropriate variables:

% No matter what, the GUI needs to be loaded
appHandle = GUI_v5_0_Main;

%% Conversions

d2r = pi/180;                                   % Degrees to radians conversion
r2d = 1/d2r;                                    % Radians to degrees conversion

%% General

% Docking
docking_face = [0 1]';                          % Docking face normal vector
docking_offset = [0.165 0.427 -pi/2]';          % Docking chaser offset [m, m, rad]
beta = pi/2-atan2(docking_offset(1), docking_offset(2)); % Docking port position vector angle [rad]

%% Control Barrier Functions
tv_CBF = 1;

% Maximum Thrust
u_max = 0.1.*[1 1 (0.3/2)]';

% Target Keep-out-Zone
r_KOZ_tar_min = [0.8; 0.42];
r_KOZ_tar_ini = [0.85; 0.85];

t_s = 4;                                        % Scaling time [s]
gamma = (0.95)^(baseRate/t_s);                  % KOZ scaling factor
eta = sqrt(0.5);                                % KOZ scaling linear tolerance [m]
zeta = 10*d2r;                                  % KOZ scaling angular tolerance [rad]

% Obstacle Keep-out-Zone
r_KOZ_obs = 0.43*[1 1]';

% Target Line-of-Sight
sensor_FOV = 40*d2r;                            % Sensor FOV [rad]
sensor_offset = [0.145-0.042 -0.0395]';         % Sensor body-fixed offset
sensor_normal = [1 0]';                         % Sensor normal vector
sensor_target = [0.0825 0.2516]';               % Desired pointing location (targed body-fixed) [m,m]

% Lyapunov Function
p_clf = 10;

V = 6*eye(3);
lambda_dock = 1;

Q_dock = [(lambda_dock^2)*eye(3) lambda_dock*V;
              lambda_dock*V           V^2];

k_dock = 5;

%% Initial Conditions

test_case = 0;

if test_case == 0
    x_RED_0 = [3 2 0 0 0 0]';
    x_BLACK_0 = [0.5 0.5 315*d2r 0.01 0.01 1*d2r]';
    x_BLUE_0 = [1.5 1.25 0 0.015 0.0075 0]';
elseif test_case == 1 
    x_RED_0 = [3.2 1.8 0 0 0 0]';
    x_BLACK_0 = [2.0 2.2 270*d2r -0.005 -0.005 -2.5*d2r]';
    x_BLUE_0 = [2.5 1.25 0 -0.01 -0.01 0]';
elseif test_case == 2
    x_RED_0 = [1.5 2 pi 0 0 0]';
    x_BLACK_0 = [2.2 0.2 270*d2r -0.005 0.005 1.5*d2r]';
    x_BLUE_0 = [2 1.2 0 -0.015 0 0]';
elseif test_case == 3
    x_RED_0 = [3 2 0 0 0 0]';
    x_BLACK_0 = [xLength/2 yLength/2 90*d2r 0 0 0]';
    x_BLUE_0 = [0 0 0 0 0 0]';
end

x_RED_0(3) = wrapAngle(rotateToFace(x_RED_0(3), x_BLACK_0(1:2)-(x_RED_0(1:2) + rotz(x_RED_0(3))*sensor_offset)) + docking_offset(3) + pi/2);

%% Pre-fill GUI

% Load Diagram
appHandle.AvailableDiagramsDropDown.Value = "ICCBF_RVD.slx";
appHandle.LoadSimulinkDiagram();

% Edit active platforms
appHandle.REDCheckBox.Value    = 1;
appHandle.BLACKCheckBox.Value  = 1;
appHandle.BLUECheckBox.Value   = 0;
appHandle.ARMCheckBox.Value    = 0;

appHandle.ConfirmSettings();  

% Edit phase durations
appHandle.DurPhase0EditField.Value = 10;
appHandle.DurPhase1EditField.Value = 10;
appHandle.DurPhase2EditField.Value = 30;

appHandle.SubPhase1EditField.Value = 100;
appHandle.SubPhase2EditField.Value = 0;
appHandle.SubPhase3EditField.Value = 0;
appHandle.SubPhase4EditField.Value = 0;

appHandle.DurPhase4EditField.Value = 0;
appHandle.DurPhase5EditField.Value = 0;

appHandle.UpdateTimes();

% Set initial conditions in the GUI
appHandle.SubAppInitialConditions.REDInitialX.Value  = x_RED_0(1);
appHandle.SubAppInitialConditions.REDInitialY.Value  = x_RED_0(2);
appHandle.SubAppInitialConditions.REDInitialTh.Value = x_RED_0(3)*r2d;
appHandle.SubAppInitialConditions.REDStartX.Value  = x_RED_0(1);
appHandle.SubAppInitialConditions.REDStartY.Value  = x_RED_0(2);
appHandle.SubAppInitialConditions.REDStartTh.Value = x_RED_0(3)*r2d;

appHandle.SubAppInitialConditions.BLACKInitialX.Value  = x_BLACK_0(1);
appHandle.SubAppInitialConditions.BLACKInitialY.Value  = x_BLACK_0(2);
appHandle.SubAppInitialConditions.BLACKInitialTh.Value = x_BLACK_0(3)*r2d;
appHandle.SubAppInitialConditions.BLACKStartX.Value  = x_BLACK_0(1);
appHandle.SubAppInitialConditions.BLACKStartY.Value  = x_BLACK_0(2);
appHandle.SubAppInitialConditions.BLACKStartTh.Value = x_BLACK_0(3)*r2d;

appHandle.SubAppInitialConditions.BLUEInitialX.Value  = x_BLUE_0(1);
appHandle.SubAppInitialConditions.BLUEInitialY.Value  = x_BLUE_0(2);
appHandle.SubAppInitialConditions.BLUEInitialTh.Value = x_BLUE_0(3)*r2d;
appHandle.SubAppInitialConditions.BLUEStartX.Value  = x_BLUE_0(1);
appHandle.SubAppInitialConditions.BLUEStartY.Value  = x_BLUE_0(2);
appHandle.SubAppInitialConditions.BLUEStartTh.Value = x_BLUE_0(3)*r2d;

appHandle.SubAppInitialConditions.UpdateInitialConditions();

%% Custom Draw Functions

function [x,y] = drawTargetKOZ(dataClass, idx)
    r_KOZ = dataClass.Target_KOZ.Data(idx,:);

    BLACK_Px = dataClass.BLACK_Px_m.Data(idx);
    BLACK_Py = dataClass.BLACK_Py_m.Data(idx);
    BLACK_Rz = dataClass.BLACK_Rz_rad.Data(idx);

    KOZ = DrawEllipse(BLACK_Px, BLACK_Py, r_KOZ(1), r_KOZ(2), BLACK_Rz);

    [x, y] = deal(KOZ(:,1), KOZ(:,2));
end  

appHandle.registerCustomDrawing("Target KOZ", @drawTargetKOZ, @patch, ...
    {'FaceColor', 'black', 'FaceAlpha', 0.1, 'EdgeColor', 'black', 'EdgeAlpha', 0.3, 'LineStyle', '--',})

function [x,y] = drawObstacleKOZ(dataClass, idx)
    r_KOZ = evalin('base', 'r_KOZ_obs');

    BLUE_Px = dataClass.BLUE_Px_m.Data(idx);
    BLUE_Py = dataClass.BLUE_Py_m.Data(idx);
    BLUE_Rz = dataClass.BLUE_Rz_rad.Data(idx);

    KOZ = DrawEllipse(BLUE_Px, BLUE_Py, r_KOZ(1), r_KOZ(2), BLUE_Rz);

    [x, y] = deal(KOZ(:,1), KOZ(:,2));
end   

appHandle.registerCustomDrawing("Obstacle KOZ", @drawObstacleKOZ, @patch, ...    
    {'FaceColor', 'blue', 'FaceAlpha', 0.1, 'EdgeColor', 'blue', 'EdgeAlpha', 0.3, 'LineStyle', '--',})

function [x,y] = drawLOS(dataClass, idx)
    RED_Px = dataClass.RED_Px_m.Data(idx);
    RED_Py = dataClass.RED_Py_m.Data(idx);
    RED_Rz = dataClass.RED_Rz_rad.Data(idx);

    sensor_FOV = evalin('base', 'sensor_FOV');
    sensor_offset = evalin('base', 'sensor_offset');
    sensor_normal = evalin('base', 'sensor_normal');

    R = [cos(RED_Rz) -sin(RED_Rz);
         sin(RED_Rz)  cos(RED_Rz)];

    r1 = [RED_Px; RED_Py] + R*sensor_offset;
    r2 = r1 + R*sensor_normal;

    [xp, yp, xm, ym] = calculateLineEndpoints(r1(1), r1(2), r2(1), r2(2), sensor_FOV, 10);

    x = [r1(1), xp, xm, r1(1)];
    y = [r1(2), yp, ym, r1(2)];
end

appHandle.registerCustomDrawing("FOV", @drawLOS, @patch, ...
    {'FaceColor', 'r', 'EdgeColor', 'r', 'EdgeAlpha', 0.2, 'FaceAlpha', 0.05, 'LineStyle', '--'})

function [x,y] = drawDesiredPosition(dataClass, idx)    
    x = dataClass.RED_Px_Desired_m.Data(idx);
    y = dataClass.RED_Py_Desired_m.Data(idx);
end

%% Place any custom variables or overwriting variables in this section

% As an example, here are the control parameters the manipulator.
% Set torque limits on joints

Tz_lim_sharm                   = .1; % Shoulder Joint [Nm]

Tz_lim_elarm                   = .1; % Elbow Joint [Nm]

Tz_lim_wrarm                   = .1; % Wrist Joint [Nm]

% Transpose Jacobian controller gains:

Kp = [0.08 0 0
      0    0.08 0
      0    0    0.002];
Kv = [0.05 0 0
      0    0.05 0
      0    0    0.005];

% Initialize the PID gains for the ARM:

Kp_sharm                       = 1.5;
Kd_sharm                       = 1.0;

Kp_elarm                       = 1.2;
Kd_elarm                       = 0.8;

Kp_wrarm                       = 1.0;
Kd_wrarm                       = 0.6;

% Define the model properties for the joint friction:
% Based on https://ieeexplore.ieee.org/document/1511048

%Shoulder
Gamma1_sh = 0.005; 
Gamma2_sh = 5;
Gamma3_sh = 40;
Gamma4_sh = 0.015; 
Gamma5_sh = 800; 
Gamma6_sh = 0.005;

%Elbow
Gamma1_el = 0.12; 
Gamma2_el = 5;
Gamma3_el = 10;
Gamma4_el = 0.039; 
Gamma5_el = 800;
Gamma6_el = 0.000001;

%Wrist
Gamma1_wr = 0.025;
Gamma2_wr = 5;
Gamma3_wr = 40;
Gamma4_wr = 0.029;
Gamma5_wr = 800; 
Gamma6_wr = 0.02;

%% This section of the code contains parameters should not be modified

% Set the PWM frequency
PWMFreq = 5; % [Hz]

%% Functions

function C = rotz(theta)
    C = [cos(theta), -sin(theta); 
         sin(theta),  cos(theta)];
end

function theta_des = rotateToFace(theta, r)
    theta_des = theta + wrapToPi(atan2(r(2),r(1)) - theta);
end

function theta_wrap = wrapAngle(theta)
    theta_wrap = atan2(sin(theta),cos(theta));
end

function [x_plus, y_plus, x_minus, y_minus] = calculateLineEndpoints(x1, y1, x2, y2, angle, lineLength)
    theta = atan2(y2 - y1, x2 - x1);

    theta_plus = theta + angle;
    theta_minus = theta - angle;

    x_plus = x2 + lineLength * cos(theta_plus);
    y_plus = y2 + lineLength * sin(theta_plus);

    x_minus = x2 + lineLength * cos(theta_minus);
    y_minus = y2 + lineLength * sin(theta_minus);
end