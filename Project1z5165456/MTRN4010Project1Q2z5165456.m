% MTRN4010 Project 1 z5165456 Scott Morris Question 2

% Create time points (0s -> 20s)
dt = 1/100;
t = 0:dt:20;
N = length(t) - 1;

% Pose for 2 different inputs
x = zeros(1, N);
y = zeros(1, N);
theta = zeros(1, N);
x1 = zeros(1, N);
y1 = zeros(1, N);
theta1 = zeros(1, N);

% Constants (speed and length)
v = 3.5; % m/s
L= 2.5; % m
L1= 3.0; % m

% Inputs
alpha = zeros(1, N);
alpha(:, 1:N/2) = deg2rad(25);
alpha(:, N/2:N) = deg2rad(-25);

% Eulers approximation for kinematic model
for i = 1:N
    x(:, i+1) = x(:, i) + dt * v * cos(theta(i));
    y(:, i+1) = y(:, i) + dt * v * sin(theta(i));
    theta(:, i+1) = theta(:, i) + dt * (v/L) * tan(alpha(i));
    x1(:, i+1) = x1(:, i) + dt * v * cos(theta1(i));
    y1(:, i+1) = y1(:, i) + dt * v * sin(theta1(i));
    theta1(:, i+1) = theta1(:, i) + dt * (v/L1) * tan(alpha(i));
end

% Plotting the position of two different vehicles
figure(1);
hold on;
title('Moving Position of vehicles from 0 to 20s');
xlabel('x position (m)');
ylabel('y position (m)');
axis([-20 20 -20 20]);
for i = 1:N
    plot(x(i), y(i), 'b--o');
    plot(x1(i), y1(i), 'r--o');
    pause(0.00001);
end


