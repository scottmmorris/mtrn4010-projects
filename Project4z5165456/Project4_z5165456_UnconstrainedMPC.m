%Project 4 (Model Predictive Control)
% Author: Subhan Khan, Date: 14/04/2021

clear all;
clc; close all;
% Initial Condition
x0 = [10;0;100;0];
y0 = [0;0;0;5000];
 % Sampling Time
dt = 0.25;
% Initial Input
u = 0;
% Ending Time
T_end = 10;
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
% Number of zeros required for gain vector
z = zeros(1,N-1);
% Calculate the gain for the MPC
K = -[1 z]*inv(G)*F;

%Simulation
for k = 1 : Nsim + 1
    % Control Action
    u(:, k) = K*x0(:,k);
    % States
    x0(:, k + 1) = A*x0(:,k) + B*u(:, k);
    % Output
    y0(:,k) = C*x0(:,k);
end

% Repeat for N = 5

x1 = [10;0;100;0];
y1 = [0;0;0;5000]; 
u1 = 0;
N2 = 5; 
Q_bar2 = kron(eye(N2),Q);
R_bar2 = kron(eye(N2),R);
Su2=zeros(N2*nx,N2*nu);
for i=1:N2
   Sx2((i-1)*nx+1:i*nx,:)=A^i;
   for j=1:i
      Su2((i-1)*nx+1:(i)*nx,(j-1)*nu+1:j*nu)=A^(i-j)*B;
   end
end
F2 = Su2'*Q_bar2*Sx2;
G2 = R_bar2 + Su2'*Q_bar2*Su2;
z2 = zeros(1,N2-1);
K2 = -[1 z2]*inv(G2)*F2;
for k = 1 : Nsim + 1
    u1(:, k) = K2*x1(:,k);
    x1(:, k + 1) = A*x1(:,k) + B*u1(:, k);
    y1(:,k) = C*x1(:,k);
end

% Plot a comparison of the two cases

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