%% ------------------ 3D Visual Simulation ------------------

addpath(genpath('/Applications/MATLAB_R2023b.app/toolbox/flypath3d'))

%load('Variables/mpc_4.mat')

% model_show('scud.mat');


coordinates = [X_k(11:-2:7, :)', X_k(1:2:5, :)' + pi/2 ];

x_lim = [-2000; 3000];
y_lim = [-2000; 3000];
z_lim = [-1200; 5000];



new_object('Rocket.mat', coordinates, 'model', 'scud.mat', 'scale', 200, ...
           'path', 'on', 'pathcolor', [.89 .0 .27], 'face', [.30 0 0]);

flypath('Rocket.mat', 'animate', 'on', 'view', [45 8], ...
        'xlim', x_lim, 'ylim', y_lim, 'zlim', z_lim);

