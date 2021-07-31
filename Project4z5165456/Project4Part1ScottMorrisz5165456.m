%Project 4 (Model Predictive Control)
% Author: Subhan Khan, Date: 14/04/2021

%% Unconstrained MPC

clear all;
clc; close all
x0 = [0;10;0;100]; % Initial Condition
y0 = [0;0;0;5000];
dt = 0.25; %sampling Time
u = 0;
T_end = 10;
t = 0:dt:T_end;
Nsim = length(t) - 1; %Total Samples
A = [-1.28 0 0.98 0;0 0 1 0;-5.43 0 -1.84 0;-128.2 -128.2 0 0]; B = [-0.3;0;-17;0]; C = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]; %States
nx=size(A,2); % number of states
nu=size(B,2); % number of inputs
N = 10; %Prediction Horizon 
R = 10; %Input penalization 
Q = diag([0 1 0 1]); %State Penalization
Q_bar = kron(eye(N),Q); %Running over the Prediction Horizon
R_bar = kron(eye(N),R); %Running over the Preidction Horizon
% Build Sx Build Su for Equation 5 (X = x_0*Sx + u*Su)
Su=zeros(N*nx,N*nu);
for i=1:N
   Sx((i-1)*nx+1:i*nx,:)=A^i; %Sx
   for j=1:i
      Su((i-1)*nx+1:(i)*nx,(j-1)*nu+1:j*nu)=A^(i-j)*B; %Su
   end
end

F = Su'*Q_bar*Sx; %Matrix F for the Equation 20-21
G = R_bar + Su'*Q_bar*Su; %Matrix G
z = zeros(1,N-1); %Zeros for gain multiplication
K = -[1 z]*inv(G)*F; %MPC Gain
%Simulation
for k = 1 : Nsim + 1
    u(:, k) = K*x0(:,k); %control action u = kx
    x0(:, k + 1) = A*x0(:,k) + B*u(:, k); %state equation
    y0(:,k) = C*x0(:,k); %output equation
end


%% Constrained MPC
x1 = [0;10;0;100]; % Initial Condition
y1 = [0;0;0;5000]; 
u1 = 0;
H = 2*G; %Cost vector H
E1 = eye(N,N); E2 = eye(N,N);
E = [E1;-E2]; %For Constraints Ez <= b
u_max = 0.262;
u_min = -0.262;
b = zeros(2*N, 1);
for i = 1:2*N
    b(i, 1) = u_max;
end

%Simulation
for k = 1 : Nsim + 1
    xp = x1(:,k); % Measure the state x0 at time instant k
    f = 2 * (F * xp); %update cost vector
    uopt = quadprog(H, f, E, b); %Compute optimal control U^*
    u1(:,k) = uopt(1); %Apply the first element u^*[0] of U to the platform
    x1(:, k + 1) = A*x1(:,k) + B*u1(:, k);
    y1(:,k) = C*x1(:,k);
end
figure(1);
subplot(221);
plot(t,u,'r:',t,u1,'b-','LineWidth',2)
xlabel('Time [sec]'); ylabel('Elevator Angle (rad)')
title('Control Action u = Kx'); legend('Unconstrained MPC', 'Constrained MPC');
grid on;
subplot(222);
plot(t,y0(2,:),'r:',t,y1(2,:),'b-','LineWidth',2)
legend('Unconstrined MPC','Constrained MPC');
xlabel('Time[sec]');ylabel('Pitch Angle (rad)')
grid on;
subplot(223);
plot(t,y0(4,:),'r:',t,y1(4,:),'b-','LineWidth',2)
legend('Unconstrined MPC','Constrained MPC');
xlabel('Time[sec]');ylabel('Altitude (m)')
grid on;