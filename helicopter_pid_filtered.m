clear; clc; close all;

% --- Parameters ---
T  = 0.1;        
gi = 0.0375;     
k  = 0.103825;   
wn = 1.27;       


numG = [38.3461, 42.9140, 36.1194];   
denG = [0.012, 1, 0];             

numF = 1; 
denF = 1;

simOpts = simset('SrcWorkspace', 'current', 'DstWorkspace', 'current');
sim('helicopter_ver19g', 120, simOpts);

% --- Evaluation ---
[J, Jcrash] = cf(t, u, r, y);

fprintf('\n=== UPDATED RESULTS ===\n');
fprintf('Total Cost (J): %.5f\n', J);
fprintf('Crash Penalty:  %.5f\n', Jcrash);

% --- Plots ---
figure;
subplot(2,1,1);
plot(t, r, 'r--', t, y, 'b', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Elevation (rad)');
legend('Reference', 'Output');

subplot(2,1,2);
plot(t, u, 'k', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Control Voltage (V)');