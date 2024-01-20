%% ----------------- Manual MPC -----------------

%clear
%close all
%clc

% Rocket Parameters

Ixx = 330.472;
Iyy = 332.721;
Izz = 334.931;

mass = 919.200;
g    =   3.711;


%% ----------------- System Matrices -----------------

n = 12;                 % Variabili
m =  4;                 % Input

% Desired State
x_r = zeros(n, 1);

% Dynamic Matrix 
%     φ     φ°    θ     θ°    ψ     ψ°    x     x°    y     y°    z     z°
Ac = [0     1     0     0     0     0     0     0     0     0     0     0;
      0     0     0     0     0     0     0     0     0     0     0     0;
      0     0     0     1     0     0     0     0     0     0     0     0;
      0     0     0     0     0     0     0     0     0     0     0     0;
      0     0     0     0     0     1     0     0     0     0     0     0;
      0     0     0     0     0     0     0     0     0     0     0     0;        
      0     0     0     0     0     0     0     1     0     0     0     0;
      0     0     0     0     0     0     0     0     0     0     0     0;
      0     0     0     0     0     0     0     0     0     1     0     0;
      0     0     0     0     g     0     0     0     0     0     0     0;
      0     0     0     0     0     0     0     0     0     0     0     1;
      0     0    -g     0     0     0     0     0     0     0     0     0];


% Input Matrix 
%     U1     U2      U3      U4    
Bc = [0      0       0       0;
      1/Ixx  0       0       0;
      0      0       0       0;
      0      1/Iyy   0       0;
      0      0       0       0;
      0      0       1/Izz   0;
      0      0       0       0;
      0      0       0       1/mass;
      0      0       0       0;
      0      0       0       0;
      0      0       0       0;
      0      0       0       0];


% System Discretization
C = eye(12);
D = 0;

format long

sys   = ss(Ac, Bc, C, D);
Ts = 0.1;                   % Tempo di Campionamento
sys_d = c2d(sys, Ts);

A = sys_d.A;
B = sys_d.B;


%% ----------------- Defining the cost function -----------------

Time = 100;                  % Prediction Horizon
tot_step = Time/Ts;        % Total Number of Step
N = 20/Ts;                   % Control Horizon

% Cost Matrix 
%          φ    φ°   θ   θ°    ψ    ψ°    x   x°   y    y°    z    z°
%Q = diag([5.0, 0.5, 10, 0.01, 5.0, 0.01, 10, 15,  1.0, 1.0,  1.0, 1.0]);

Q = diag([10.0, 0.5, 10, 0.01, 5.0, 0.01, 100, 100,  20, 20,  20, 20]);

% Wheight Matrix 
%          U1    U2    U3    U4    
R = diag([0.01, 0.01, 0.01, 0.01]);

% Terminal Cost
Q_N = 30*Q;


% Cost Function 
[T, S]     = my_system (A, B, N);
[Q_X, R_U] = my_hessian(Q, Q_N, R, N );

H = 2*(R_U + S'*Q_X*S);        H = (H+H')/2;        
F = 2*(S'*Q_X*T);
Y = 2*(Q + T'*Q_X*T);

%fun = @(z, xk)  1/2* z'*H*z  +  xk'*F'*z  +  1/2* xk'*Y*xk;


%% ----------------- Upper and Lower Bounds -----------------

lb = -inf*ones(N*m, 1);       % Lower bounds initialized to -inf
ub =  inf*ones(N*m, 1);       % Upper bounds initialized to +inf

% Control Variables Limits 
for i = 1:N
    % Torque 
    lb(1 + (i-1)*m: 3 + (i-1)*m)   = -50;
    ub(1 + (i-1)*m: 3 + (i-1)*m)   = +50;
    
    % Thrust Force
    lb(4 + (i-1)*m)    =   0;
    ub(4 + (i-1)*m)    =  10000;
end


%% ----------------- Simulation ------------------

% Initial State
%      φ    φ°    θ        θ°     ψ      ψ°     x       x°     y     y°     z       z°
x_0 = [1    0    0.8863    0    -0.49    0    4000    -75    1000    40    1500    100]';
xk = x_0;


% State Variable Limits 
%         φ      φ°    θ       θ°     ψ       ψ°      x       x°     y      y°     z      z°
lower = [-pi   -inf   -pi    -inf    -pi    -inf      0     -inf   -inf   -inf   -inf   -inf]';
upper = -lower;

massimi = upper;
minimi  = lower;
for i = 1:N-1    
    minimi  = [minimi;  lower];
    massimi = [massimi; upper];
end
W = [massimi; -minimi];

X_k = zeros(n, tot_step);        X_k(1:n, 1) = x_0;
z_k = zeros(m*N, tot_step); 

for i = 1:tot_step  
    fprintf("Iteration %d of %d \n", i, tot_step)
    
    % State Variable Constraints (angles between -pi and +pi)
    V = [S; -S];
    rhs = W - [T; -T]*xk ;
    
    % Quadratic Optimization 
    f = xk'*F';
    z_k(:, i) = quadprog(H, f, [], [], [], [], lb, ub);

    % Updating State Variables
    X_k(:, i+1) = A*xk + B*z_k(1:m, i);
    xk = X_k(:, i+1);
end


%% ----------------- Plotting Results ------------------
close all

load('Variables/mpc_5.mat')

figure(1)           % Control
    subplot(2,1,1)
    plot(z_k(1:3, :)', 'LineWidth', 2);
    xlabel('Tempo (s)');
    ylabel('Torque(Nm)');
    grid on;
    legend('U1','U2', 'U3');

    subplot(2,1,2)
    plot(z_k(4, :)', 'LineWidth', 2);
    xlabel('Tempo (s)');
    ylabel('Force(N)');
    legend('U4');
    grid on;

    sgtitle('Evoluzione ingressi di controllo');
   

figure(2)           % State
    subplot(2,2,1)
    plot(X_k(7:2:11, :)', 'LineWidth', 2);
    xlabel('Tempo (s)');
    ylabel('Position (m)');
    grid on;
    legend('x', 'y', 'z');
    
    subplot(2,2,2)
    plot(X_k(8:2:12, :)', 'LineWidth', 2);   
    xlabel('Tempo (s)');
    ylabel('Speed (m/s)');
    grid on;
    legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$', 'interpreter', 'latex');
    
    subplot(2,2,3)
    plot(X_k(1:2:5, :)', 'LineWidth', 2);   
    xlabel('Tempo (s)');
    ylabel('Angular Position (rad)');
    grid on;
    legend('φ', 'θ', 'ψ');
    
    subplot(2,2,4)
    plot(X_k(2:2:6, :)', 'LineWidth', 2);   
    xlabel('Tempo (s)');
    ylabel('Angular Velocity (rad/s)');
    grid on;
    legend('$\dot{\phi}$', '$\dot{\theta}$', '$\dot{\psi}$', 'interpreter', 'latex');
    
    sgtitle('Evoluzione degli stati del sistema');

save("Variables/mpc_6", "X_k", "z_k")



