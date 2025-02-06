# MATLAB CODE 
Here is the matlab code for the step test:

```matlab
clc;
clear;

% Define system parameters
num = [0.755];
den = [1, 13.87, 34.91];
gs = tf(num, den);

num_lc = [1, 0.08774];
den_lc = [1, 0.00911];
gs_lc_1 = tf(num_lc, den_lc);

num_lc = [1, 16.05];
den_lc = [1, 25.1];
gs_lc_2 = tf(num_lc, den_lc);

k = 59.449;
gs_lc = series( gs_lc_1, gs_lc_2);

gs_top = series(gs_lc_1, gs_lc_2)
gs_top = series( k, gs_top)
gs_total = series( gs_top, gs);
gs_total = feedback(gs_total, 1);

% Display step info
disp('Step Info using step');
stepinfo(gs_total)

% Time vector for simulation
t = 0:0.01:10;  % Simulate for 10 seconds with 0.01s step size

% Input signal: Start at 2, then step to 3 at t = 4
u = 2 * ones(size(t));  % Initial value is 2
u(t >= 4) = 3;          % Step to 3 at t = 4

% Simulate system response
[y, t_out] = lsim(gs_total, u, t);

% Plot the response
figure;
plot(t_out, y, 'b', 'LineWidth', 1.5); hold on;
plot(t_out, u, 'r--', 'LineWidth', 1.2);  % Overlay the input signal
title('Step Response Analysis');
xlabel('Time (s)');
ylabel('Output');
legend('System Response', 'Input Signal (Step)');
grid on;

% Analyze the step response from 2 to 3
step_data = y(t >= 4);  % Focus on data after the step
time_data = t_out(t >= 4);  % Corresponding time values

% Calculate step information for the step from 2 to 3
step_info = stepinfo(step_data, time_data-4);  % Subtract 2 to analyze the step from 2 to 3

% Display step info
disp('Step Info (from 2 to 3):');
disp(step_info);

% Discretize gs_lc
gs_lc_discrete = c2d(gs_top, 0.01, 'tustin')
disp('Discrete-time Transfer Function (Tustin method):');
disp(gs_lc_discrete);
pole(gs_lc_discrete)

```

