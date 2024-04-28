%% Plant
% Create 's'
s = tf('s');

% System parameters
m = 10;
k = 0.5;

% System transfer function
G = 1/(s*(m*s+k));

%% Controller synthesis - multiple configurations

% Error weight function:
% High low frequency gain to achieve good rejection of constant
% dusturbance
% Define configurations
configs = {
    makeweight(100, 1, 0.1); % Configuration 1
    makeweight(100, 0.1, 0.1)   % Configuration 2
    makeweight(100, 0.05, 0.1)   % Configuration 2
};

% Control effort and controlled variable weights:
% Big high frequency gain to limit control effort at high frequency and
% achieve good robustness
W2 = makeweight(0, 100, 1.1);
W3 = makeweight(0, 100, 1.1);

% Display transfer functions:
W2_tf = tf(W2)
W3_tf = tf(W3)

% Preallocate arrays for storage
K = cell(size(configs));
CL = cell(size(configs));
gamma = cell(size(configs));
results = struct([]);

% Loop through configurations
for i = 1:length(configs)
    W1 = configs{i}; % Use the ith configuration
    W1_tf = tf(W1)

    % Plot Bode diagram for weights
    figure;
    bodemag(W1, W2, W3);
    set(findall(gcf,'type','line'),'LineWidth',2);
    yline(0,'--');
    legend({'W1', 'W2', 'W3', '0 dB'}, 'FontSize', 12);
    grid on;

    % Compute K using mixed-sensitivity loop shaping
    [K{i}, CL{i}, gamma{i}] = mixsyn(G, W1, W2, W3);

    % Find K's transfer function
    K_tf = tf(K{i})

    % Open loop function's Bode diagram to check stability margins
    figure;
    margin(K_tf*G); % Plot Bode diagram with gain and phase margins
    set(findall(gcf,'type','line'),'LineWidth',2);
    grid on;

    % Run the model with the ith configuration
    simOut = sim("H_inf_pos_control");

    % Access the signals from out.logsout
    results(i).r = simOut.logsout.get('r').Values;
    results(i).F = simOut.logsout.get('F').Values;
    results(i).z = simOut.logsout.get('z').Values;

end

%% Plotting Results
figure;
% Subplot for response
subplot(2, 1, 1);
hold on;
for i = 1:length(configs)
    plot(results(i).z.Time, results(i).z.Data, 'LineWidth', 2);
end
plot(results(1).r.Time, results(1).r.Data, '--', 'LineWidth', 2);
hold off;
ylabel('Position (m)');
legend(arrayfun(@(x) sprintf('Position - Conf %d', x), 1:length(configs), 'UniformOutput', false), 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Subplot for control effort
subplot(2, 1, 2);
hold on;
for i = 1:length(configs)
    plot(results(i).F.Time, results(i).F.Data, 'LineWidth', 2);
end
hold off;
ylabel('Force (N)');
xlabel('Time (s)');
legend(arrayfun(@(x) sprintf('Force - Conf %d', x), 1:length(configs), 'UniformOutput', false), 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;