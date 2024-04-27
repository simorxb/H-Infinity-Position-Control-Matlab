%% Plant
% Create 's'
s = tf('s');

% System parameters
m = 10;
k = 0.5;

% System transfer function
G = 1/(s*(m*s+k));

%% Controller synthesis - configuration 1

% Error weight function:
% High low frequency gain to achieve good rejection of constant
% dusturbance
W1 = makeweight(10000, [0.1 50], 10);

% Control effort and controlled variable weights:
% Big high frequency gain to limit control effort at high frequency and
% achieve good robustness
W2 = makeweight(0, 5, 2);
W3 = makeweight(0, 5, 2);

% Plot Bode diagram for weights
figure;
bodemag(W1, W2, W3);
yline(0,'--');
legend({'W1', 'W2', 'W3', '0 dB'}, 'FontSize', 12);
grid on;

% Use mixed-sensitivity loop shaping to compute K
[K,CL,gamma] = mixsyn(G,W1,W2,W3);

% Find K's transfer function
K_tf = tf(K);

% Open loop function's Bode diagram to check stability margins
figure;
bode(K_tf*G);
grid on;

%% Test

% Run the model

out = sim("H_inf_pos_control");

% Access the signals from out.logsout

r = out.logsout.get('r').Values.Data;
t_r = out.logsout.get('r').Values.Time;

F = out.logsout.get('F').Values.Data;
t_F = out.logsout.get('F').Values.Time;

z = out.logsout.get('z').Values.Data;
t_z = out.logsout.get('z').Values.Time;

% Create the figure
figure;

% Subplot for response
subplot(2, 1, 1);
plot(t_z, z, 'LineWidth', 2);
hold on;
plot(t_r, r, '--', 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Position (m)');
legend({'Position - Conf 1', 'Setpoint'}, 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Subplot for control effort
subplot(2, 1, 2);
plot(t_F, F, 'LineWidth', 2);
%xlabel('Time (s)');
ylabel('Force (N)');
legend({'Force - Conf 1'}, 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;