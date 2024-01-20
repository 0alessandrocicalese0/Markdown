clear all
close all
clc
yalmip('clear')
addpath(genpath('flypath3d'));
addpath(genpath('GPAD-master'));
addpath(genpath('/Applications/MATLAB_R2023b.app/toolbox/YALMIP-master'))

%% Parametri razzo
Ixx = 330.472;
Iyy = 332.721;
Izz = 334.931;
m   = 919.200;
g   =   3.711;

%% Matrici SYS
%     φ     φ°    θ     θ°    ψ     ψ°    x     x°    y     y°    z     z°
A = [ 0     1     0     0     0     0     0     0     0     0     0     0;
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

    
B =   [0     0     0     0;
       1/Ixx 0     0     0;
       0     0     0     0;
       0     1/Iyy 0     0;
       0     0     0     0;
       0     0     1/Izz 0;
       0     0     0     0;
       0     0     0     1/m;
       0     0     0     0;
       0     0     0     0;
       0     0     0     0;
       0     0     0     0];

C = eye(12);

D = 0;

% Discretizzazione
sys   = ss(A,B,C,D);
Ts    = 0.01;
sys_d = c2d(sys, Ts,'zoh');

% MPC
nx = 12;
nu =  4;
N  = 20;
Nc =  5;


% Definizione dello stato finale

x_desired = zeros(12,1);

% Stato iniziale
%     φ    φ°    θ        θ°     ψ      ψ°    x       x°     y     y°     z       z°
x0 = [0    0    0.8863    0    -0.49    0    1500    -75    500    40    2000    100]';


% Dichiarazione delle variabili
u = sdpvar(repmat(nu, 1, N),     repmat(1, 1, N));
x = sdpvar(repmat(nx, 1, N + 1), repmat(1, 1, N + 1));
epsS = sdpvar(1);
epsH = sdpvar(1);


% Definizione della funzione costo
%       φ     φ°    θ     θ°    ψ     ψ°    x     x°    y     y°    z     z°
Q   = [ 5     0     0     0     0     0     0     0     0     0     0     0
        0     0.5   0     0     0     0     0     0     0     0     0     0;
        0     0     10    0     0     0     0     0     0     0     0     0;
        0     0     0     0.01  0     0     0     0     0     0     0     0;
        0     0     0     0     5     0     0     0     0     0     0     0;
        0     0     0     0     0     0.01  0     0     0     0     0     0;
        0     0     0     0     0     0     10    0     0     0     0     0;
        0     0     0     0     0     0     0     15    0     0     0     0;
        0     0     0     0     0     0     0     0     1     0     0     0;
        0     0     0     0     0     0     0     0     0     1     0     0;
        0     0     0     0     0     0     0     0     0     0     1     0;
        0     0     0     0     0     0     0     0     0     0     0     1 ];

% Matrice dei pesi 

R = [0.01   0       0       0;
     0      0.01    0       0;
     0      0       0.01    0;
     0      0       0       0.1];

R = 0.1 * R;

% Funzione obiettivo
objective = 0;
for k = 1:N
    objective = objective + (x{k} - x_desired)' * Q * (x{k} - x_desired) + (u{k})' * 0 * (u{k});
end
objective = objective + epsH^2 * 0.01 +  epsS^2;

% Vincoli
constr    = [];
for k = 1:Nc
    constr = [constr, x{k+1} == sys_d.A*x{k} + sys_d.B*u{k}];
    constr = [constr, abs(x{k}(1)) <= 2*pi + epsS];
    constr = [constr, abs(x{k}(3)) <= 2*pi + epsS];
    constr = [constr, abs(x{k}(5)) <= 2*pi + epsS];

    constr = [constr,  abs(u{k}(3)) <= 300   + epsH];
    constr = [constr,       u{k}(4) <= 10000 + epsH];
    constr = [constr,  abs(u{k}(2)) <= 300   + epsH];
end

% Definizione il controller
controller = optimizer(constr, objective,[], x{1}, [u{:}]);

%inizio simulazione
x = x0;

implementedU = [];
implementedX = x;

tsim = 300;

exU = 0;
for i=1:Ts:tsim
    i
    U = controller{x};
    x = sys_d.A * x + sys_d.B * U(:,1);
    implementedX=[implementedX, x];
    implementedU=[implementedU, U(:,1)];
end

implementedU;
implementedX;

t = 0:Ts:tsim-1;
t=[t,t(end)+Ts];
t1 = 0:Ts:tsim-1; 

%Plot
figure(1)
    subplot(2,1,1)
    plot(t1,implementedU(1,:));
    xlabel('Tempo (s)');
    ylabel('Torque(Nm)');
    grid on;
    hold on
    plot(t1,implementedU(2,:));
    xlabel('Tempo (s)');
    ylabel('Torque(Nm)');
    plot(t1,implementedU(3,:));
    xlabel('Tempo (s)');
    ylabel('Torque(Nm)');
    legend('U1','U2', 'U3');
    subplot(2,1,2)
    plot(t1,implementedU(4,:));
    xlabel('Tempo (s)');
    ylabel('Force(N)');
    grid on;
 sgtitle('Evoluzione ingressi di controllo');

figure(2)
    subplot(2,2,1)
    plot(t, implementedX([7 9 11],:));
    xlabel('Tempo (s)');
    ylabel('Posizione (m)');
    grid on;
    legend('x', 'y', 'z');
    subplot(2,2,2)
    plot(t, implementedX([8 10 12],:));
    xlabel('Tempo (s)');
    ylabel('Velocità (m/s)');
    grid on;
    legend('x', 'y', 'z');
    subplot(2,2,3)
    plot(t, implementedX([1 3 5],:));
    xlabel('Tempo (s)');
    ylabel('Posizione angolare (rad)');
    grid on;
    legend('φ', 'θ', 'ψ');
    subplot(2,2,4)
    plot(t, implementedX([2 4 6],:));
    xlabel('Tempo (s)');
    ylabel('Velocità angolare (rad/s)');
    grid on;
    legend('φ', 'θ', 'ψ');
sgtitle('Evoluzione degli stati del sistema');


close all

%% GRAFICO 3D ANIMATO

x_graph = implementedX(11,:);   
y_graph = implementedX(9,:);    
z_graph = implementedX(7,:);   
x_lim = [min(x_graph)-200 max(x_graph)+200];
y_lim = [min(y_graph)-200 max(y_graph)+200];
z_lim = [min(z_graph)-200 max(z_graph)+200];

pitch_graph = implementedX(3,:)+pi/2;
yaw_graph = implementedX(5,:);
roll_graph = implementedX(1,:);

trajectory = [x_graph; y_graph; z_graph; pitch_graph; yaw_graph; roll_graph]';
new_object('tbm.mat', trajectory, 'model', 'scud.mat', 'scale', 30, ...
           'path', 'on', 'pathcolor', [.89 .0 .27], 'face',[.30 0 0]);


flypath('tbm.mat', 'animate', 'on', 'step', 200, 'axis', 'on',...
        'axiscolor', [0 0 0], 'color', [1 1 1], 'font', 'Georgia', ...
        'fontsize', 6,'view',[45 8],'window',[2400 2000],...
        'xlim', x_lim, 'ylim', y_lim, 'zlim', z_lim, ...
        'output', 'tbm_example.png','dpi',600);
