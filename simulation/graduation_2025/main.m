% clear; clc; close all;

%% --- Parameters ---
rho = 1000;         % water density (kg/m^3)
V = 1.5;            % forward velocity (m/s)
S = 0.004;          % fin area (m^2)
CL_alpha = 6.28;    % lift curve slope (per rad, approx 2π)
lf = 0.45;          % moment arm (m)
Itheta = 2.83;     % pitch inertia (kg*m^2)
I_psi = 0.165;     % yaw inertia (kg*m^2)
params = [rho V S CL_alpha lf Itheta];
m = 18;            % mass (kg)
d_theta = 5;
d_psi = 1.5;
k_z = 0.5;             % vertical drag cof
v_bias = 0.03;         % buoyancy drift

%% --- Response Evaluation ---
out = sim('dynamic_simulation');

z_ts = out.logsout.get('z').Values;
t_z = z_ts.Time;
z = z_ts.Data;

info_z = stepinfo(z, t_z, 1);

figure
plot(t_z, z, 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Depth (m)')
title('Depth Response')

theta_ts = out.logsout.get('theta').Values;
t_theta = theta_ts.Time;
theta = theta_ts.Data;

figure
plot(theta_ts.Time, theta_ts.Data, 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Pitch Angle (rad)')
title('Pitch Angle Response')

info_theta = stepinfo(theta_ts.Data, theta_ts.Time);

psi_ts = out.logsout.get('psi').Values;
t_psi = psi_ts.Time;
psi = psi_ts.Data;

info_psi = stepinfo(psi, t_psi, 0.523);

figure
plot(psi_ts.Time, psi_ts.Data, 'LineWidth', 1.5)
grid on
xlabel('Time (s)')
ylabel('Yaw Angle (rad)')
title('Yaw Angle Response')
