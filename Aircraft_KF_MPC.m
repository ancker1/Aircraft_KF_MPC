clear all; clc; close all; format compact;

%% System modeling
T = 4; %[s]
dt = 0.05;
nSteps  = T/dt;
Nx = 3;
Nu = 1;
Ny = 1;
N = 50; % Time horizon

% system description
A = [   -0.313 56.7 0
        -0.0139 -0.426 0
        0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1]; 

Fk = eye(3)+A*dt;
Gk = B*dt;


%% Initialization
u = deg2rad(1);

x0     = [0; 0; deg2rad(10)];
xplusArray = [];
z      = zeros((Nx+Nu)*N,1);
z(1:Nx)= x0; 
y      = 0;
yArray = [];
uArray = [];
xArray = [];


%% Constraint
 xlow  = [-deg2rad(0) -deg2rad(30) -deg2rad(45) ]';
 ulow  = -deg2rad(30);
 xhigh = [deg2rad(30) deg2rad(30) deg2rad(45) ]';
 uhigh = deg2rad(30);
 
 lb = [repmat(xlow, N, 1) ; ulow *ones(N*Nu,1)];
 ub = [repmat(xhigh, N, 1); uhigh*ones(N*Nu,1)];

%% Weights
    % We do not care about the first two states [alpha, q, theta]
q = 10000;
Q = [0 0 0; 0 0 0; 0 0 q];
    % Only one input
R = 0.1;

%% Input reference
th_ref = deg2rad(20);
x_ref = [0; 0; th_ref];

%% Simulation
    % Define G and c
G = zeros(Nx*N + Nu*N);
f = zeros(Nx*N+ Nu*N,1);
Aeq_1 = eye(N*Nx);
Aeq_2 = zeros(N*Nx, N*Nu);

    % Define the matrix idx
idx_x = 1;
idx_u = 1 + N*Nx;
for i = 1:N
        % PLace the Q weight
    G(idx_x:idx_x+Nx-1, idx_x:idx_x+Nx-1) = Q;
    f(idx_x:idx_x+Nx-1) = -Q*x_ref;
    idx_x = idx_x + Nx;
        % Place the R weight
    G(idx_u:idx_u+Nu-1, idx_u:idx_u+Nu-1) = R;
    f(idx_u:idx_u+Nu-1) = 0;
    idx_u = idx_u + Nu;
        % Do the Aeq
    Aeq_2((i-1)*Nx+1:i*Nx, (i-1)*Nu+1:i*Nu) = Aeq_2((i-1)*Nx+1:i*Nx, (i-1)*Nu+1:i*Nu) + -Gk;
    if i ~= N
        Aeq_1(i*Nx+1:(i+1)*Nx, (i-1)*Nx+1:i*Nx) = -Fk;
    end
end
Aeq = [Aeq_1 Aeq_2];

%% Kalman
P = eye(Nx)  * 1e-3; 
Q = eye(Nx)  * 1e-8;  % Noise - model
R = eye(Nu)  * deg2rad(0.1);  % Noise - sensor

%%
x = x0;
uk = 0;
for i = 1:nSteps
        
    xArray = [xArray x];            % Prediction - with noise
    xplusArray = [xplusArray x0];   % Update estimate of state
    uArray = [uArray uk];           % Control
    yArray = [yArray y];            % Measurement - with noise
    
    beq = zeros(N*Nx, 1);
    beq(1:Nx, 1) = Fk*x0; 
    
    options = optimoptions('quadprog','Display','off');
    z   = quadprog(G,f,[],[],Aeq,beq,lb,ub,[],options);
    uk = z(N*Nx+1);
    
    % Simulate the system
    x = Fk*x0 + Gk*uk + sqrt(Q)*randn(Nx, 1);
    y = C*x + sqrt(R)*randn;
    
        % ------------- KALMAN State -------------------------
    %y = Fk*x0 + Gk*uk + sqrt(R)*randn; % output
    % predict
    xmin  = Fk*x0 + Gk*uk;
    P   = Fk*P*Fk' + Q;
    % update
    K   = P*C'*inv(C*P*C'+R);
    x0  = xmin + K*(y-C*xmin);
    P   = (eye(Nx)-K*C)*P;
    
end

%% Plotting
figure(1)
subplot(2,1,1)
plot(dt:dt:T,rad2deg(xplusArray(3,:)),'o:')
hold on
title('State vs time')
plot(dt:dt:T,rad2deg(yArray),'o:')
yline(rad2deg(th_ref),'r')
legend('Estimated state', 'Measured state', 'Reference')
grid on;
xlabel('Time [s]')
ylabel('State: Pitch angle, theta [deg]')
xlim([0 4])
hold off

subplot(2,1,2)
stairs(dt:dt:T,rad2deg(uArray), 'o-')
title('Control input vs time')
xlabel('Time [s]')
ylabel('Elevator deflection angle [deg]')
grid on;
xlim([0 4])

figure(2)
subplot(2,1,1)
plot(dt:dt:T,rad2deg(xArray(3,:)),'o:')
legend('x')
title('Predicted State [th] vs time')
ylim([-45 45])
grid on;
subplot(2,1,2)
stairs(dt:dt:T,rad2deg(uArray), 'o-')
title('Control input [rad] vs time')
grid on;

%%
figure(3)
subplot(2,1,1)
plot(rad2deg(z(3:3:N*Nx)),'o:')
%legend('\theta')
title('Pitch angle versus time step')
%ylim([-45 45])
xlabel('Time step')
ylabel('\theta [degree]')
grid on;
subplot(2,1,2)
stairs(rad2deg(z(N*Nx+1:end)), 'o-')
title('Control input versus time step')
grid on;
xlabel('Time step')
ylabel('\delta [degree]')