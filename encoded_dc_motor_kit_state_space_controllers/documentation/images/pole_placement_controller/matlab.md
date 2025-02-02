# MATLAB CODE 
Here is the matlab code for the step test:

```matlab
clc;
clear;

%% DESIGN OF SERVO SYSTEMS
%% Design of Type 1 Servo System when the Plant Has An Integrator.
num = 0.755;
den = [1, 13.87, 34.91];
A = [0        1;
     -34.91  -13.87];

B = [0  ; 0.755];

C = [1 0];

P = [A     B;
     -C    0];
rank(P); % has rank n+1 thus the system is completely state controllable

A_hat = [ A          zeros(2, 1); 
         -C          0];
B_hat = [B;
         0];
J = [(-2+j*3) (-2-j*3) 1];

Khat = acker(A_hat, B_hat, J);
K = [Khat(1) Khat(2)];
K_1 = Khat(3);

AA = [A-B*K   B*K_1;
      -C      0];
BB = [0; 0; 1];
CC = [C 0];
DD = [0];

% Simulate the step response from 2 to 3
t = 0:0.02:6;
[y, x, t] = step(AA, BB, CC, DD, 1, t);

% Scale and shift the output to simulate a step from 2 to 3
y_scaled = 2 + y; % Shift the step response from 2 to 3
x1 = x(:, 1); % Velocity (first state)
x2 = x(:, 2); % Acceleration (second state)
x3 = x(:, 3); % Integrator state (third state)

% Compute state feedback and integrator contributions
state_feedback = x1 * K(1) + x2 * K(2); % State feedback contribution
integrator_contribution = x3 * K_1;     % Integrator contribution
control_output = state_feedback + integrator_contribution; % Total control output

% Plot the results
figure;

% Plot state feedback contribution
subplot(4, 1, 1);
plot(t, state_feedback, 'LineWidth', 1.5);
grid on;
title('State Feedback Contribution');
xlabel('Time (s)');
ylabel('State Feedback');

% Plot integrator contribution
subplot(4, 1, 2);
plot(t, integrator_contribution, 'LineWidth', 1.5);
grid on;
title('Integrator Contribution');
xlabel('Time (s)');
ylabel('Integrator');

% Plot total control output
subplot(4, 1, 3);
plot(t, control_output, 'LineWidth', 1.5);
grid on;
title('Total Control Output');
xlabel('Time (s)');
ylabel('Control Output');

% Plot total output
subplot(4, 1, 4);
plot(t, y_scaled, 'LineWidth', 1.5);
grid on;
title('Total Control Output');
xlabel('Time (s)');
ylabel('Control Output');

% Plot the scaled step response (from 2 to 3)
%figure;
%plot(t, y_scaled, 'LineWidth', 1.5);
%grid on;
%title('Step Response (from 2 to 3)');
%xlabel('Time (s)');
%ylabel('Output');

% Extract step response information from y_scaled data
stepInfo = stepinfo(y_scaled, t, 'RiseTimeThreshold', [0.1 0.9], 'SettlingTimeThreshold', 0.02);
disp('Step Response Information from y_scaled data:');
disp(stepInfo);

% Manually calculate some metrics if needed
steadyStateValue = y_scaled(end); % Steady-state value
peakValue = max(y_scaled); % Peak value
overshoot = 100 * (peakValue - steadyStateValue) / steadyStateValue; % Percentage overshoot
disp(['Steady-State Value: ', num2str(steadyStateValue)]);
disp(['Peak Value: ', num2str(peakValue)]);
disp(['Overshoot: ', num2str(overshoot), '%']);
```

