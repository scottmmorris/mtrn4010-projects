% MTRN4010 Project 1 z5165456 Scott Morris Question 1

% Create time points (0s -> 7s)
dt = 1/1000;
t = 0:dt:7;
N = length(t) - 1;

% Angular position and velocity for 2 different inputs
X = zeros(2, 7001);
X2 = zeros(2, 7001);
X(1, 1) = deg2rad(110);
X2(1, 1) = deg2rad(110);

% Input value (constant)
u = 0;
u2 = 3;

% Constants
A = 110; % rad/s^2
B = 2.2; % /s
C = 1.1; % rad/(s^2 * V) 

% Eulers Approximation for kinematic model
for i = 1:7000
    X(:, i+1) = X(:, i) + dt * [X(2, i);-A * sin(X(1, i)) - B * X(2,i) + C * u];
    X2(:, i+1) = X2(:, i) + dt * [X2(2, i);-A * sin(X2(1, i)) - B * X2(2,i) + C * u2];
end

% Plotting angular position (Fig 1) and angular velocity (Fig 2)
figure(1);
hold on;
plot(t, rad2deg(X(1,:)), 'b');
plot(t, rad2deg(X2(1,:)), 'r');
title('Angular position from time 0 to time 7s');
xlabel('Time (s)');
ylabel('Angular position (deg)');

figure(2);
hold on;
plot(t, rad2deg(X(2,:)), 'b');
plot(t, rad2deg(X2(2,:)), 'r');
title('Angular velocity from time 0 to time 7s');
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');