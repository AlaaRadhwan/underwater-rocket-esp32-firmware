function dx = rocket_pitch_dynamics(t, x, params, pid)

%% Unpack states
theta = x(1);
theta_dot = x(2);

%% Unpack parameters
rho = params(1);
V = params(2);
S = params(3);
CL_alpha = params(4);
lf = params(5);
Itheta = params(6);

%% Unpack PID gains
Kp = pid(1);
Ki = pid(2);
Kd = pid(3);

%% Persistent variables for integral and previous error
persistent eint prev_e last_t

if isempty(eint)
    eint = 0;
    prev_e = 0;
    last_t = t;
end

%% Desired pitch angle
theta_ref = 0;  % keeping robot level

%% PID error
e = theta_ref - theta;

%% Time step
dt = max(t - last_t, 1e-6);

%% PID components
eint = eint + e*dt;
edot = (e - prev_e) / dt;

u = Kp*e + Ki*eint + Kd*edot;   % fin command (radians)

prev_e = e;
last_t = t;

if mod(round(t*100),50) == 0
    fprintf("t=%.2f  e=%.3f  u=%.3f\n", t, e, u);
end


%% Hydrodynamic Lift (per fin)
alpha = u; % assume fin deflection = angle of attack
L_fin = 0.5 * rho * V^2 * S * CL_alpha * alpha;

%% Total pitch moment
Mtheta = 2 * L_fin * lf;

%% Pitch dynamics
theta_ddot = Mtheta / Itheta;

%% Return derivatives
dx = [theta_dot;
      theta_ddot];
