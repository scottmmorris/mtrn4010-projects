%Project 4 (Model Predictive Control) Constrained MPC
% Author: Scott Morris

clear all;
clc; close all;
% Initial Condition
x0 = [0;10;0;100];
y0 = [0;0;0;5000];
 % Sampling Time
dt = 0.25;
% Initial Input
u = 0;
% Ending Time
T_end = 30;
% Time Vector
t = 0:dt:T_end;
% Length of Time Vector
Nsim = length(t) - 1;
% State Matrices
A = [-1.28 0 0.98 0;0 0 1 0;-5.43 0 -1.84 0;-128.2 -128.2 0 0];
B = [-0.3;0;-17;0];
C = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% Number of states (in our case, 4)
nx=size(A,2);
% Number of inputs (in our case, 1)
nu=size(B,2);
% Prediction horizon size (i.e. how many sampling intervals forward should
% we predict
N = 10;
% Input penalty value over the prediction horizon
R = 10;
R_bar = kron(eye(N),R);
% State penalty values over the prediction horizon
Q = diag([0,1,0,1]);
Q_bar = kron(eye(N),Q);
% Calculate Sx and Su (X = x_0*Sx + u*Su)
Su=zeros(N*nx,N*nu);
for i=1:N
   Sx((i-1)*nx+1:i*nx,:)=A^i;
   for j=1:i
      Su((i-1)*nx+1:(i)*nx,(j-1)*nu+1:j*nu)=A^(i-j)*B;
   end
end
% Claculate the cost function relating to the states
F = Su'*Q_bar*Sx;
% Claculate the cost function relating to the inputs
G = R_bar + Su'*Q_bar*Su;
% Reformulate G cost vector due to applied constraints
H = 2*G;

% Setup constraints for input
E1 = eye(N,N); E2 = eye(N,N);
E = [E1;-E2];
u_max = 0.262;
u_min = -0.262;
b = zeros(2*N, 1);
for i = 1:2*N
    b(i, 1) = u_max;
end

% Simulation
for k = 1 : Nsim + 1
    % Get the current state of the system
    xp = x0(:,k);
    % Update the cost function using the current state
    f = 2 * (F * xp);
    % Use quadprog to solve the cost minimisation (H & f) and obtain control input
    % for the supplied constraints (E & b)
    uopt = quadprog(H, f, E, b);
    u(:,k) = uopt(1);
    % Update state
    x0(:, k + 1) = A*x0(:,k) + B*u(:, k);
    % Update output
    y0(:,k) = C*x0(:,k);
end

% Repeat for N = 5

x1 = [0;10;0;100];
y1 = [0;0;0;5000]; 
u1 = 0;
N = 5;
Q_bar1 = kron(eye(N),Q);
R_bar1 = kron(eye(N),R);
Su1=zeros(N*nx,N*nu);
for i=1:N
   Sx1((i-1)*nx+1:i*nx,:)=A^i;
   for j=1:i
      Su1((i-1)*nx+1:(i)*nx,(j-1)*nu+1:j*nu)=A^(i-j)*B;
   end
end
F1 = Su1'*Q_bar1*Sx1;
G1 = R_bar1 + Su1'*Q_bar1*Su1;
H1 = 2*G1;
E11 = eye(N,N); E21 = eye(N,N);
E111 = [E11;-E21];
b1 = zeros(2*N, 1);
for i = 1:2*N
    b1(i, 1) = u_max;
end
for k = 1 : Nsim + 1
    xp = x1(:,k);
    f1 = 2 * (F1 * xp);
    uopt = quadprog(H1, f1, E111, b1);
    u1(:,k) = uopt(1);
    x1(:, k + 1) = A*x1(:,k) + B*u1(:, k);
    y1(:,k) = C*x1(:,k);
end

% Plot comparison of different cases

figure(1);
subplot(221);
plot(t,u,'r',t,u1,'b');
xlabel('Time [sec]'); ylabel('Elevator Angle (rad)')
title('Control Action u = Kx'); legend('N = 10', 'N = 5');
grid on;
subplot(222);
plot(t,y0(2,:),'r',t,y1(2,:),'b');
legend('N = 10', 'N = 5');
xlabel('Time[sec]');ylabel('Pitch Angle (rad)')
grid on;
subplot(223);
plot(t,y0(4,:),'r',t,y1(4,:),'b');
legend('N = 10', 'N = 5');
xlabel('Time[sec]');ylabel('Altitude (m)')
grid on;