% pid_design.m
% Filtered PID design for the 3DOF helicopter elevation axis
% using pole-placement coefficient matching.
%
% Control Engineering (CE-1542), Taif University, 2026
% Author: Reema Khalid Al-boqami
%
% Requires: Control System Toolbox

clear; clc; close all;

%% Plant: P(s) = b0 / (s^2 + a1*s + a0)
a1 = 0.09525;
a0 = 1.613;
b0 = 0.1675;
P  = tf(b0, [1 a1 a0]);

%% Design specifications
zeta  = 0.6901;   % damping ratio for 5% overshoot
wn    = 1.1;      % natural frequency (rad/s)
alpha = 5.0;      % third real pole at s = -alpha
tau   = 0.012;    % filter time constant (s)

%% Gains from matching (s^2 + 2*zeta*wn*s + wn^2)(s + alpha)
Kd = (2*zeta*wn + alpha - a1) / b0;
Kp = (wn^2 + 2*zeta*wn*alpha - a0) / b0;
Ki = (alpha * wn^2) / b0;

fprintf('Kd = %.4f\nKp = %.4f\nKi = %.4f\n', Kd, Kp, Ki);

%% Filtered PID: Gc(s) = (Kd*s^2 + Kp*s + Ki) / (s*(tau*s + 1))
numG = [Kd Kp Ki];
denG = [tau 1 0];
Gc   = tf(numG, denG);

%% Closed loop
T = feedback(Gc * P, 1);
disp('Closed-loop transfer function T(s):');
T

disp('Closed-loop poles:');
disp(pole(T));

%% Step response
info = stepinfo(T);
fprintf('Overshoot: %.2f %%\nSettling time (2%%): %.2f s\n', ...
        info.Overshoot, info.SettlingTime);

figure;
step(T, 12);
grid on;
title('Unit step response, linear closed-loop model');
ylabel('Elevation (normalized)');
